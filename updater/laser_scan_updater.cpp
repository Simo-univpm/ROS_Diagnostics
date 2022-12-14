/*

  Il codice implementa il nodo updater per il laser scanner della carrozzina. 
  Raccoglie i dati dal topic /scan ed effettua un rilevamento a soglia per individuare potenziali guasti dovuti a valori anomali dei sensori.
  I messaggi diagnostici vengono pubblicati sul topic /diagnostics.

*/

#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>

/*

  Variabili globali per la memorizzazione dei valori attuali dei sensori e delle soglie.

*/

float angle_min = 0.0;
float angle_max = 0.0;
float angle_increment = 0.0;
float scan_time = 0.0;
float range_min = 0.0;
float range_max = 0.0;
std::vector<float> ranges;
std::vector<float> intensities;

float angle_min_threshold = 0.0;
float angle_max_threshold = 0.0;
float angle_increment_threshold = 0.0;
float scan_time_threshold = 0.0;
float range_min_threshold = 0.0;
float range_max_threshold = 0.0;

/*

  La scanCallback permette di salvare nelle variabili globali i valori correnti dei sensori.

*/

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  angle_min = msg->angle_min;
  angle_max = msg->angle_max;
  angle_increment = msg->angle_increment;
  scan_time = msg->scan_time;
  range_min = msg->range_min;
  range_max = msg->range_max;
  ranges = msg->ranges;
  intensities = msg->intensities;

}


/*

  La funzione angle_min_diagnostic implementa il rilevamento a soglia per la variabile angle_min creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void angle_min_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(angle_min > angle_min_threshold) stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "angle_min ERROR! val: %f, threshold: %f", angle_min, angle_min_threshold);
  else stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "angle_min OK val: %f, threshold: %f", angle_min, angle_min_threshold);


  stat.add("Diagnostica di esempio", "Controllo del valore di angle_min del laser");
  std::ostringstream string;
  string << angle_min;
  stat.addf("angle_min:", string.str().c_str());

}

/*

  La funzione angle_max_diagnostic implementa il rilevamento a soglia per la variabile angle_max creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void angle_max_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(angle_max > angle_max_threshold) stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "angle_max ERROR! val: %f, threshold: %f", angle_max, angle_max_threshold);
  else stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "angle_max OK val: %f, threshold: %f", angle_min, angle_min_threshold);


  stat.add("Diagnostica di esempio", "Controllo del valore di angle_max del laser");
  std::ostringstream string;
  string << angle_max;
  stat.addf("angle_max:", string.str().c_str());

}

/*

  La funzione angle_increment_diagnostic implementa il rilevamento a soglia per la variabile angle_increment creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void angle_increment_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(angle_increment > angle_increment_threshold) stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "angle_increment ERROR! val: %f, threshold: %f", angle_increment, angle_increment_threshold);
  else stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "angle_increment OK val: %f, threshold: %f", angle_increment, angle_increment_threshold);


  stat.add("Diagnostica di esempio", "Controllo del valore di angle_increment del laser");
  std::ostringstream string;
  string << angle_increment;
  stat.addf("angle_increment:", string.str().c_str());

}

/*

  La funzione scan_time_diagnostic implementa il rilevamento a soglia per la variabile scan_time creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void scan_time_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(scan_time > scan_time_threshold) stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "scan_time ERROR! val: %f, threshold: %f", scan_time, scan_time_threshold);
  else stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "scan_time OK val: %f, threshold: %f", scan_time, scan_time_threshold);


  stat.add("Diagnostica di esempio", "Controllo del valore di scan_time del laser");
  std::ostringstream string;
  string << scan_time;
  stat.addf("scan_time:", string.str().c_str());

}

/*

  La funzione range_min_diagnostic implementa il rilevamento a soglia per la variabile range_min creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void range_min_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(range_min > range_min_threshold) stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "range_min ERROR! val: %f, threshold: %f", range_min, range_min_threshold);
  else stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "range_min OK val: %f, threshold: %f", range_min, range_min_threshold);


  stat.add("Diagnostica di esempio", "Controllo del valore di range_min del laser");
  std::ostringstream string;
  string << range_min;
  stat.addf("range_min:", string.str().c_str());

}

/*

  La funzione range_max_diagnostic implementa il rilevamento a soglia per la variabile range_max creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void range_max_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(range_max > range_max_threshold) stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "range_max ERROR! val: %f, threshold: %f", range_max, range_max_threshold);
  else stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "range_max OK val: %f, threshold: %f", range_max, range_max_threshold);


  stat.add("Diagnostica di esempio", "Controllo del valore di range_max del laser");
  std::ostringstream string;
  string << range_max;
  stat.addf("range_max:", string.str().c_str());

}

/*

  La funzione ranges_diagnostic implementa il rilevamento a soglia per la variabile ranges creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.

  Il messaggio contiene una tabella che riporta soltato i valori di ranges fuori soglia.
  
*/

void ranges_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  for(int i=0; i < ranges.size(); i++){
    if (ranges[i] < range_min || ranges[i] > range_max ){
           
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "range fuori soglia. Range_min: %f, Range_max: %f", range_min,range_max);
      stat.add("Diagnostica di esempio", "Valore i-esimo del vettore ranges");
      std::ostringstream ranges_string, index_string;
      ranges_string << ranges[i];
      index_string << i;
      stat.addf("ranges["+ index_string.str() + "]", ranges_string.str().c_str());
      index_string.str("");
      ranges_string.str("");
    
    }else if (ranges[i] > range_min && ranges[i] < range_max){
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "range OK val: %f, > %f && < %f", ranges[i], range_min, range_max);
    }
  }  
}

/*

  La funzione intensities_diagnostic implementa il rilevamento a soglia per la variabile intensities creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.

  Il messaggio contiene una tabella che riporta soltato i valori di intensities fuori soglia.
  
*/

void intensities_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  for(int i=0; i < intensities.size(); i++){
    if (intensities[i] < range_min || intensities[i] > range_max ){
           
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "intensity fuori soglia. Range_min: %f, Range_max: %f", range_min,range_max);
      stat.add("Diagnostica di esempio", "Valore i-esimo del vettore intensities");
      std::ostringstream intensities_string, index_string;
      intensities_string << intensities[i];
      index_string << i;
      stat.addf("intensities["+ index_string.str() + "]", intensities_string.str().c_str());
      index_string.str("");
      intensities_string.str("");
    
    }else if (intensities[i] > range_min && intensities[i] < range_max){
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "range OK val: %f, > %f && < %f", intensities[i], range_min, range_max);
    }
  }  

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_scan_diagnostic");

  ros::NodeHandle nh_scan;
  diagnostic_updater::Updater scan_updater;
  scan_updater.setHardwareID("/scan");

  /*
    
    Caricamento delle variabili globali con le soglie ottenute dai valori raccolti dal file YAML di configurazione.

  */

  nh_scan.getParam("/scan_params/scan_thresholds/angle_min_threshold", angle_min_threshold);
  nh_scan.getParam("/scan_params/scan_thresholds/angle_max_threshold", angle_max_threshold);
  nh_scan.getParam("/scan_params/scan_thresholds/angle_increment_threshold", angle_increment_threshold);
  nh_scan.getParam("/scan_params/scan_thresholds/scan_time_threshold", scan_time_threshold);
  nh_scan.getParam("/scan_params/scan_thresholds/range_min_threshold", range_min_threshold);
  nh_scan.getParam("/scan_params/scan_thresholds/range_max_threshold", range_max_threshold);

  /*

    Il metodo subscribe permette di sottoscriversi al topic /scan per estrarne i messaggi. Il metodo richiama la funzione scanCallback.

  */

  ros::Subscriber sub = nh_scan.subscribe("/scan", 1000, scanCallback);

  /*
  
    Il metodo add permette la creazione del messaggio diagnostico invocando la funzione che effettuerà il rilevamento a soglia.

  */
  scan_updater.add("Funzione di diagnostica di angle_min", angle_min_diagnostic);
  scan_updater.add("Funzione di diagnostica di angle_max", angle_max_diagnostic);
  scan_updater.add("Funzione di diagnostica di angle_increment", angle_increment_diagnostic);
  scan_updater.add("Funzione di diagnostica di scan_time", scan_time_diagnostic);
  scan_updater.add("Funzione di diagnostica di range_min", range_min_diagnostic);
  scan_updater.add("Funzione di diagnostica di range_max", range_max_diagnostic);
  scan_updater.add("Funzione di diagnostica di ranges", ranges_diagnostic);
  scan_updater.add("Funzione di diagnostica di intensities", intensities_diagnostic);


  while (nh_scan.ok())
  {
    std_msgs::Bool msg;
    ros::Duration(0.1).sleep();
    ros::spinOnce();
   
    msg.data = false;
    
    scan_updater.update();
  }

  return 0; 
}
