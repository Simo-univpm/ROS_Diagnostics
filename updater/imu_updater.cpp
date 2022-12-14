/*

  Il codice implementa il nodo updater per l'IMU della carrozzina. 
  Raccoglie i dati dal topic /imu/data ed effettua un rilevamento a soglia per individuare potenziali guasti dovuti a valori anomali dei sensori.
  I messaggi diagnostici vengono pubblicati sul topic /diagnostics.

*/

#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <sstream>
#include <sensor_msgs/Imu.h>

/*

  Variabili globali per la memorizzazione dei valori attuali dei sensori e delle soglie.

*/


float x_imu_orientation_threshold = 0.0;
float x_imu_orientation = 0.0;
float y_imu_orientation_threshold = 0.0;
float y_imu_orientation = 0.0;
float z_imu_orientation_threshold = 0.0;
float z_imu_orientation = 0.0;
float w_imu_orientation_threshold = 0.0;
float w_imu_orientation = 0.0;

float x_imu_angular_velocity_threshold = 0.0;
float x_imu_angular_velocity = 0.0;
float y_imu_angular_velocity_threshold = 0.0;
float y_imu_angular_velocity = 0.0;
float z_imu_angular_velocity_threshold = 0.0;
float z_imu_angular_velocity = 0.0;

float x_imu_linear_acceleration_threshold = 0.0;
float x_imu_linear_acceleration = 0.0;
float y_imu_linear_acceleration_threshold = 0.0;
float y_imu_linear_acceleration = 0.0;
float z_imu_linear_acceleration_threshold = 0.0;
float z_imu_linear_acceleration = 0.0;

/*

  La imuCallback permette di salvare nelle variabili globali i valori correnti dei sensori.

*/

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    x_imu_orientation = msg->orientation.x;
    y_imu_orientation = msg->orientation.y;
    z_imu_orientation = msg->orientation.z;
    w_imu_orientation = msg->orientation.w;

    x_imu_angular_velocity = msg->angular_velocity.x;
    y_imu_angular_velocity = msg->angular_velocity.y;
    z_imu_angular_velocity = msg->angular_velocity.z;

    x_imu_linear_acceleration = msg->linear_acceleration.x;
    y_imu_linear_acceleration = msg->linear_acceleration.y;
    z_imu_linear_acceleration = msg->linear_acceleration.z;

}

/*

  La funzione x_imu_orientation_diagnostic implementa il rilevamento a soglia per la variabile x_imu_orientation creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void x_imu_orientation_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(x_imu_orientation > x_imu_orientation_threshold) 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_imu_orientation e' fuori soglia! Valore limite: %f", x_imu_orientation_threshold);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_imu_orientation: %f", x_imu_orientation);

  stat.add("Diagnostica IMU", "Valore");    
  std::ostringstream x_imu_orientation_string;                              // creazione di una variabile di tipo ostringstream
  x_imu_orientation_string << x_imu_orientation;                            // con l'operatore << si carica la stringa all'interno della variabile ostringstream
  stat.addf("x_imu_orientation", x_imu_orientation_string.str().c_str());   // si effettua il cast al tipo c_str come richiesto dal metodo addf
}

/*

  La funzione y_imu_orientation_diagnostic implementa il rilevamento a soglia per la variabile y_imu_orientation creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void y_imu_orientation_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(y_imu_orientation > y_imu_orientation_threshold)
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_orientation e' fuori soglia! Valore limite: %f", y_imu_orientation_threshold);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_orientation: %f", y_imu_orientation);

  stat.add("Diagnostica IMU", "Valore");
  std::ostringstream y_imu_orientation_string;
  y_imu_orientation_string << y_imu_orientation;
  stat.addf("y_imu_orientation", y_imu_orientation_string.str().c_str());
}

/*

  La funzione z_imu_orientation_diagnostic implementa il rilevamento a soglia per la variabile z_imu_orientation creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void z_imu_orientation_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
  if(z_imu_orientation > z_imu_orientation_threshold) 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_orientation e' fuori soglia! Valore limite: %f", z_imu_orientation_threshold);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_orientation: %f", z_imu_orientation);

  stat.add("Diagnostica IMU", "Valore");
  std::ostringstream z_imu_orientation_string;
  z_imu_orientation_string << z_imu_orientation;
  stat.addf("z_imu_orientation", z_imu_orientation_string.str().c_str());
}

/*

  La funzione w_imu_orientation_diagnostic implementa il rilevamento a soglia per la variabile w_imu_orientation creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void w_imu_orientation_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(w_imu_orientation > w_imu_orientation_threshold) 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore w_orientation e' fuori soglia! Valore limite: %f", w_imu_orientation_threshold);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, w_orientation: %f", w_imu_orientation);

  stat.add("Diagnostica IMU", "Valore");
  std::ostringstream w_imu_orientation_string;
  w_imu_orientation_string << w_imu_orientation;
  stat.addf("w_imu_orientation", w_imu_orientation_string.str().c_str());
}

/*

  La funzione x_imu_angular_velocity_diagnostic implementa il rilevamento a soglia per la variabile x_imu_angular_velocity creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void x_imu_angular_velocity_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(x_imu_angular_velocity > x_imu_angular_velocity_threshold)
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_imu_angular_velocity e' fuori soglia! Valore limite: %f", x_imu_angular_velocity_threshold);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_imu_angular_velocity: %f", x_imu_angular_velocity);

  stat.add("Diagnostica IMU", "Valore");
  std::ostringstream x_imu_angular_velocity_string;
  x_imu_angular_velocity_string << x_imu_angular_velocity;
  stat.addf("x_imu_angular_velocity", x_imu_angular_velocity_string.str().c_str());
}

/*

  La funzione y_imu_angular_velocity_diagnostic implementa il rilevamento a soglia per la variabile y_imu_angular_velocity creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void y_imu_angular_velocity_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(y_imu_angular_velocity > y_imu_angular_velocity_threshold) 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_imu_angular_velocity e' fuori soglia! Valore limite: %f", y_imu_angular_velocity_threshold);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_imu_angular_velocity: %f", y_imu_angular_velocity);

  stat.add("Diagnostica IMU", "Valore");
  std::ostringstream y_imu_angular_velocity_string;
  y_imu_angular_velocity_string << y_imu_angular_velocity;
  stat.addf("y_imu_angular_velocity", y_imu_angular_velocity_string.str().c_str());
}

/*

  La funzione z_imu_angular_velocity_diagnostic implementa il rilevamento a soglia per la variabile z_imu_angular_velocity creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void z_imu_angular_velocity_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(z_imu_angular_velocity > z_imu_angular_velocity_threshold) 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_imu_angular_velocity e' fuori soglia! Valore limite: %f", z_imu_angular_velocity_threshold);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_imu_angular_velocity: %f", z_imu_angular_velocity);

  stat.add("Diagnostica IMU", "Valore");
  std::ostringstream z_imu_angular_velocity_string;
  z_imu_angular_velocity_string << z_imu_angular_velocity;
  stat.addf("z_imu_angular_velocity", z_imu_angular_velocity_string.str().c_str());
}

/*

  La funzione x_imu_linear_acceleration_diagnostic implementa il rilevamento a soglia per la variabile x_imu_linear_acceleration creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void x_imu_linear_acceleration_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(x_imu_linear_acceleration > x_imu_linear_acceleration_threshold) 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_imu_linear_acceleration e' fuori soglia! Valore limite: %f", x_imu_linear_acceleration_threshold);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_imu_linear_acceleration: %f", x_imu_linear_acceleration);

  stat.add("Diagnostica IMU", "Valore");
  std::ostringstream x_imu_linear_acceleration_string;
  x_imu_linear_acceleration_string << x_imu_linear_acceleration;
  stat.addf("x_imu_linear_acceleration", x_imu_linear_acceleration_string.str().c_str());
}

/*

  La funzione y_imu_linear_acceleration_diagnostic implementa il rilevamento a soglia per la variabile y_imu_linear_acceleration creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void y_imu_linear_acceleration_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(y_imu_linear_acceleration > y_imu_linear_acceleration_threshold) 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_imu_linear_acceleration e' fuori soglia! Valore limite: %f", y_imu_linear_acceleration_threshold);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_imu_linear_acceleration: %f", y_imu_linear_acceleration);

  stat.add("Diagnostica IMU", "Valore");
  std::ostringstream y_imu_linear_acceleration_string;
  y_imu_linear_acceleration_string << y_imu_linear_acceleration;
  stat.addf("y_imu_linear_acceleration", y_imu_linear_acceleration_string.str().c_str());
}

/*

  La funzione z_imu_linear_acceleration_diagnostic implementa il rilevamento a soglia per la variabile z_imu_linear_acceleration creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void z_imu_linear_acceleration_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

  if(z_imu_linear_acceleration > z_imu_linear_acceleration_threshold) 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_imu_linear_acceleration e' fuori soglia! Valore limite: %f", z_imu_linear_acceleration_threshold);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_imu_linear_acceleration: %f", z_imu_linear_acceleration);

  stat.add("Diagnostica IMU", "Valore");
  std::ostringstream z_imu_linear_acceleration_string;
  z_imu_linear_acceleration_string << z_imu_linear_acceleration;
  stat.addf("z_imu_linear_acceleration", z_imu_linear_acceleration_string.str().c_str());
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_diagnostic");

  ros::NodeHandle nh_imu;
  diagnostic_updater::Updater imu_updater;
  imu_updater.setHardwareID("/imu/data");

  /*
    
    Caricamento delle variabili globali con le soglie ottenute dai valori raccolti dal file YAML di configurazione.

  */

  nh_imu.getParam("/imu_params/orientation_threshold/x_threshold", x_imu_orientation_threshold);
  nh_imu.getParam("/imu_params/orientation_threshold/y_threshold", y_imu_orientation_threshold);
  nh_imu.getParam("/imu_params/orientation_threshold/z_threshold", z_imu_orientation_threshold);
  nh_imu.getParam("/imu_params/orientation_threshold/w_threshold", w_imu_orientation_threshold);

  nh_imu.getParam("/imu_params/angular_velocity_threshold/x_threshold", x_imu_angular_velocity_threshold);
  nh_imu.getParam("/imu_params/angular_velocity_threshold/y_threshold", y_imu_angular_velocity_threshold);
  nh_imu.getParam("/imu_params/angular_velocity_threshold/z_threshold", z_imu_angular_velocity_threshold);

  nh_imu.getParam("/imu_params/linear_acceleration_threshold/x_threshold", x_imu_linear_acceleration_threshold);
  nh_imu.getParam("/imu_params/linear_acceleration_threshold/y_threshold", y_imu_linear_acceleration_threshold);
  nh_imu.getParam("/imu_params/linear_acceleration_threshold/z_threshold", z_imu_linear_acceleration_threshold);

  /*

    Il metodo subscribe permette di sottoscriversi al topic /imu/data per estrarne i messaggi. Il metodo richiama la funzione imuCallback.

  */

  ros::Subscriber sub = nh_imu.subscribe("/imu/data", 1000, imuCallback);

  /*
  
    Il metodo add permette la creazione del messaggio diagnostico invocando la funzione che effettuerà il rilevamento a soglia.

  */

  imu_updater.add("Funzione di diagnostica della IMU orientation", x_imu_orientation_diagnostic);
  imu_updater.add("Funzione di diagnostica della IMU orientation", y_imu_orientation_diagnostic);
  imu_updater.add("Funzione di diagnostica della IMU orientation", z_imu_orientation_diagnostic);
  imu_updater.add("Funzione di diagnostica della IMU orientation", w_imu_orientation_diagnostic);

  imu_updater.add("Funzione di diagnostica della IMU angular velocity", x_imu_angular_velocity_diagnostic);
  imu_updater.add("Funzione di diagnostica della IMU angular velocity", y_imu_angular_velocity_diagnostic);
  imu_updater.add("Funzione di diagnostica della IMU angular velocity", z_imu_angular_velocity_diagnostic);

  imu_updater.add("Funzione di diagnostica della IMU linear acceleration", x_imu_linear_acceleration_diagnostic);
  imu_updater.add("Funzione di diagnostica della IMU linear acceleration", y_imu_linear_acceleration_diagnostic);
  imu_updater.add("Funzione di diagnostica della IMU linear acceleration", z_imu_linear_acceleration_diagnostic);


  while (nh_imu.ok())
  {
    std_msgs::Bool msg;
    ros::Duration(0.1).sleep();
    ros::spinOnce();
   
    msg.data = false;
    
    imu_updater.update();
  }

  return 0; 
}
