/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file main
 * main prepares the monocular image "sensor" and starts the main loop, sending one Mat image at a time.
 * Previously initializes the system when building the SLAM object, passing as arguments the routes to the vocabulary and configuration,,
 * and the MONOCULAR mode.
 */


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <System.h>
#include <Viewer.h>
#include <Tracking.h>
#include <FrameDrawer.h>
#include <Serializer.h>
#include <Video.h>


using namespace std;

ORB_SLAM2::System *Sistema;

int main(int argc, char **argv){

	cout	<< "Starting ORB-SLAM. Command line:" << endl
			<< "os1 [yaml configuration file [path to video file]] \ nNo arguments to use webcam, with configuration in webcam.yaml" << endl;

    // Parameters of the command line

    char* rutaConfiguracion = NULL;
    char* rutaVideo = NULL;
	char archivoConfiguracionWebcamPorDefecto[] = "webcam.yaml";	// Default configuration, for webcam.

	switch(argc){
	case 1:	// No arguments, default webcam and webcam.yaml as configuration
		rutaConfiguracion = archivoConfiguracionWebcamPorDefecto;
		cout << "Sin argumentos, webcam con esta configuración: " << rutaConfiguracion << endl;
		break;

	case 2:	// An argument, configuration file NOT IMPLEMENTED
		rutaConfiguracion = argv[1];
		break;

	case 3:	// Two arguments, configuration file and video path
		rutaConfiguracion = argv[1];
		rutaVideo = argv[2];
		cout << "Two arguments: " << rutaConfiguracion << ", " << rutaVideo << endl;
		break;

	}

	// Initialize the SLAM system.
    // MMy binary file version with the vocabulary, which loads much faster because it avoids parsing.
    ORB_SLAM2::System SLAM("orbVoc.bin", rutaConfiguracion,ORB_SLAM2::System::MONOCULAR,true);

    // Global pointer to the singleton system
    Sistema = &SLAM;

    // Entrance image
    cv::Mat im;

    ORB_SLAM2::Viewer* visor = SLAM.mpViewer;

    // Start the Video thread
    ORB_SLAM2::Video video;
    new thread(&ORB_SLAM2::Video::Run, &video);

	// Indicates if the video entry corresponds to a file, and therefore its time base is controllable
	bool videoEsArchivo = rutaVideo;
    if(videoEsArchivo){
		video.abrirVideo(rutaVideo);
		visor->setDuracion(video.cantidadCuadros);
	}else{
		// There are no parameters, there is no video, only webcam.
		video.abrirCamara();
		visor->setDuracion();
	}

    while(true){

        // Read new picture, control the time of the video
    	if(video.flujo == ORB_SLAM2::Video::VIDEO || video.flujo == ORB_SLAM2::Video::VIDEO_RT){
    		if(visor->tiempoAlterado){
				// The user moved the trackbar: you have to change the frame.
				video.setCuadroPos(visor->tiempo);
				visor->tiempoAlterado = false;	// Lower the signal.
			} else if(visor->tiempoReversa && !visor->videoPausado){
				// The movie goes backwards
				if(video.posCuadro<2){
					// If you reach the beginning of the video, start again forward
					video.setCuadroPos(0);
					visor->tiempoReversa = false;
				}
			}
    	}

    	// t1 is in seconds, with double precision. The loop for initialization takes between 1 and 2 hundredths of a second.
    	// std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        if(video.imagenDisponible)
        	SLAM.TrackMonocular(
        		video.getImagen(
        			visor->videoPausado? 0 :
        			visor->tiempoReversa? -1 :
        			1
        		),
				(double)video.posCuadro	// timestamp
			);


    	// See if there is a signal to load the map, which must be done from this thread
    	if(visor->cargarMapa){
    		visor->cargarMapa = false;

    		// The subsequent reset requires that LocalMapping is not paused.
    		SLAM.mpLocalMapper->Release();

        	// Clean the map of all the singletons
    		SLAM.mpTracker->Reset();
    		// At this point the system is reset.

    	    // Wait for LocalMapping and Viewer to stop
    		SLAM.mpLocalMapper->RequestStop();
    		SLAM.mpViewer	  ->RequestStop();

        	char charchivo[1024];
        	FILE *f = popen("zenity --file-selection", "r");
        	fgets(charchivo, 1024, f);

        	if(charchivo[0]){
            	while(!SLAM.mpLocalMapper->isStopped()) usleep(1000);
    			while(!SLAM.mpViewer	 ->isStopped()) usleep(1000);

				std::string nombreArchivo(charchivo);
				nombreArchivo.pop_back();	// Remove the final \ n
				cout << "Abriendo archivo " << nombreArchivo << endl;

				SLAM.serializer->mapLoad(nombreArchivo);
				cout << "Mapa cargado." << endl;

        	}
			SLAM.mpTracker->mState = ORB_SLAM2::Tracking::LOST;

			// Reactive viewer. It does not reactivate the mapper, because the system remains in only tracking after loading.
			SLAM.mpViewer->Release();

			// By the doubts, it is what Tracking does after the state passes to LOST.
			// Since he has a mutex, just call it after viewer.release.
			SLAM.mpFrameDrawer->Update(SLAM.mpTracker);
    	}
    	if(visor->guardarMapa){
    		visor->guardarMapa = false;

    	    // Wait for LocalMapping to stop
    		SLAM.mpLocalMapper->RequestStop();
    		SLAM.mpViewer	  ->RequestStop();	// It does not seem necessary to save, but only to load, because when saving the map is not modified.

        	//char archivo[] = "mapa.bin";
        	char charchivo[1024];
        	FILE *f = popen("zenity --file-selection --save --confirm-overwrite --filename=mapa.osMap", "r");
        	fgets(charchivo, 1024, f);

        	if(charchivo[0]){
        		while(!SLAM.mpLocalMapper->isStopped()) usleep(1000);
        		while(!SLAM.mpViewer	 ->isStopped()) usleep(1000);

				std::string nombreArchivo(charchivo);
				nombreArchivo.pop_back();	// Remove the final \ n
				cout << "Guardando archivo " << nombreArchivo << endl;

            	SLAM.serializer->mapSave(nombreArchivo);
            	cout << "Mapa guardado." << endl;
        	}

        	// Reactive viewer. It does not reactivate the mapper, because the system remains in only tracking after loading.
        	SLAM.mpViewer->Release();
    	}


    	// Open a video file
    	if(visor->abrirVideo){
    		visor->abrirVideo = false;
        	char charchivo[1024];
        	FILE *f = popen("zenity --file-selection", "r");
        	fgets(charchivo, 1024, f);
        	if(charchivo[0]){
				std::string nombreArchivo(charchivo);
				nombreArchivo.pop_back();	// Remove the final \ n
				cout << "Abriendo video " << nombreArchivo << endl;
				video.abrirVideo(nombreArchivo);
				cout << "Video abierto." << endl;

				// Open calibration file, which is the same as the configuration but only the camera parameters are read
				Sistema->mpTracker->ChangeCalibration(
						// You have to change the extension of the name of the video file to .yaml
						(nombreArchivo.erase(nombreArchivo.find_last_of('.'))).append(".yaml")
				);
        	}

    	}

    	// Open a webcam
    	if(visor->abrirCamara){
    		visor->abrirCamara = false;
    		video.abrirCamara();
    		Sistema->mpTracker->ChangeCalibration(archivoConfiguracionWebcamPorDefecto);//"webcan.yaml");
    	}

    	/*
        // Stop stopwatch to measure processing duration
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(std::chrono::steady_clock::now() - t1).count();

        // Delay for 30 fps, 0.033 s period
        if(ttrack < 0.033)
        	usleep((0.033-ttrack)*1e6);
        */

    }
    cout << "Invoke shutdown..." << endl;

    // Stop all threads
    SLAM.Shutdown();

    cout << "Finished." << endl;

    return 0;
}
