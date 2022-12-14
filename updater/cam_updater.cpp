/*

  Il codice implementa il nodo updater per la camera della carrozzina. 
  Raccoglie i dati dal topic /camera_sensor/image_raw ed effettua un rilevamento a soglia per individuare potenziali guasti dovuti a valori anomali della camera.
  I messaggi diagnostici vengono pubblicati sul topic /diagnostics.

*/

#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include "sensor_msgs/Image.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>

/*

  Variabili globali per la memorizzazione dei valori attuali dei sensori e delle soglie.

*/

std::vector<uint8_t> raw_image;
uint32_t step = 0;

/*

  La callback permette di salvare nelle variabili globali i valori correnti della camera.

*/

void callback(const sensor_msgs::Image::ConstPtr& msg)
{

  raw_image = msg -> data;
  step = msg -> step;

}

/*

  La funzione camera_sensor_diagnostic va a verificare la grandezza del vettore contenente i dati
  pubblicati della camera: se la grandezza dell'array ricevuto non è coerente con quanto specificato
  nella documentazione verrà pubblicato sul topic /diagnostics un messaggio di warning.
  
*/

void camera_sensor_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  int raw_image_size = step * step;

  if(raw_image.size() != raw_image_size){
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "immagine ricevuta correttamente");
    stat.add("Diagnostica di esempio", "Controllo USB camera");
    stat.addf("USB cam:", "OK");
  } else {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "immagine corrotta o camera sconnessa");
    stat.add("Diagnostica di esempio", "Controllo USB camera");
    stat.addf("USB cam:", "WARN");
  }


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_sensor_diagnostic");

  ros::NodeHandle nh_cam;
  ros::Subscriber sub = nh_cam.subscribe("/camera_sensor/image_raw", 1000, callback);
  diagnostic_updater::Updater cam_updater;
  

  cam_updater.setHardwareID("/camera_sensor");
  cam_updater.add("Funzione di diagnostica di camera_sensor", camera_sensor_diagnostic);


  while (nh_cam.ok())
  {
    std_msgs::Bool msg;
    ros::Duration(0.1).sleep();
    ros::spinOnce();
   
    msg.data = false;
    
    cam_updater.update();
  }

  return 0; 
}
