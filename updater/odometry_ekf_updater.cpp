/*

  Il codice implementa il nodo updater per l'odometry EKF della carrozzina. 
  Raccoglie i dati dal topic /odometry/ekf_local ed effettua un rilevamento a soglia per individuare potenziali guasti dovuti a valori anomali dei sensori.
  I messaggi diagnostici vengono pubblicati sul topic /diagnostics.

*/

#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <nav_msgs/Odometry.h>
#include <sstream>

/*

  Variabili globali per la memorizzazione dei valori attuali dei sensori e delle soglie.

*/

float x_ekf_pose_position = 0.0;
float x_ekf_pose_position_threshold = 0.0;
float y_ekf_pose_position = 0.0;
float y_ekf_pose_position_threshold = 0.0;
float z_ekf_pose_position = 0.0;
float z_ekf_pose_position_threshold = 0.0;

float x_ekf_pose_orientation = 0.0;
float x_ekf_pose_orientation_threshold = 0.0;
float y_ekf_pose_orientation = 0.0;
float y_ekf_pose_orientation_threshold = 0.0;
float z_ekf_pose_orientation = 0.0;
float z_ekf_pose_orientation_threshold = 0.0;
float w_ekf_pose_orientation = 0.0;
float w_ekf_pose_orientation_threshold = 0.0;

float x_ekf_twist_linear = 0.0;
float x_ekf_twist_linear_threshold = 0.0;
float y_ekf_twist_linear = 0.0;
float y_ekf_twist_linear_threshold = 0.0;
float z_ekf_twist_linear = 0.0;
float z_ekf_twist_linear_threshold = 0.0;

float x_ekf_twist_angular = 0.0;
float x_ekf_twist_angular_threshold = 0.0;
float y_ekf_twist_angular = 0.0;
float y_ekf_twist_angular_threshold = 0.0;
float z_ekf_twist_angular = 0.0;
float z_ekf_twist_angular_threshold = 0.0;

/*

  La ekfPosePositionCallback permette di salvare nelle variabili globali i valori correnti dei sensori.

*/

void ekfPosePositionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  x_ekf_pose_position = msg->pose.pose.position.x;
  y_ekf_pose_position = msg->pose.pose.position.y;
  z_ekf_pose_position = msg->pose.pose.position.z;

  x_ekf_pose_orientation = msg->pose.pose.orientation.x;
  y_ekf_pose_orientation = msg->pose.pose.orientation.y;
  z_ekf_pose_orientation = msg->pose.pose.orientation.z;

  x_ekf_twist_linear = msg->twist.twist.linear.x;
  y_ekf_twist_linear = msg->twist.twist.linear.y;
  z_ekf_twist_linear = msg->twist.twist.linear.z;

  x_ekf_twist_angular = msg->twist.twist.angular.x;
  y_ekf_twist_angular = msg->twist.twist.angular.y;
  z_ekf_twist_angular = msg->twist.twist.angular.z;

}

/*

  La funzione xEkfPosePositionDiagostic implementa il rilevamento a soglia per la variabile x_ekf_pose_position creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void xEkfPosePositionDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (x_ekf_pose_position > x_ekf_pose_position_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_ekf_pose_position e' fuori soglia! Valore limite: %f", x_ekf_pose_position_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_ekf_pose_position: %f", x_ekf_pose_position);

  
  stat.add("Diagnostica Ekf Pose Position", "Valore");
  std::ostringstream x_ekf_pose_position_string;
  x_ekf_pose_position_string << x_ekf_pose_position;
  stat.addf("x Ekf Pose Position", x_ekf_pose_position_string.str().c_str());
}

/*

  La funzione yEkfPosePositionDiagostic implementa il rilevamento a soglia per la variabile y_ekf_pose_position creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void yEkfPosePositionDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (y_ekf_pose_position > y_ekf_pose_position_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_ekf_pose_position e' fuori soglia! Valore limite: %f", y_ekf_pose_position_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_ekf_pose_position: %f", y_ekf_pose_position);

  
  stat.add("Diagnostica Pose Position", "Valore");
  std::ostringstream y_ekf_pose_position_string;
  y_ekf_pose_position_string << y_ekf_pose_position;
  stat.addf("y Ekf Pose Position", y_ekf_pose_position_string.str().c_str());
}

/*

  La funzione zEkfPosePositionDiagostic implementa il rilevamento a soglia per la variabile z_ekf_pose_position creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void zEkfPosePositionDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (z_ekf_pose_position > z_ekf_pose_position_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_ekf_pose_position e' fuori soglia! Valore limite: %f", z_ekf_pose_position_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_ekf_pose_position: %f", z_ekf_pose_position);

  
  stat.add("Diagnostica Ekf Pose Position", "Valore");
  std::ostringstream z_ekf_pose_position_string;
  z_ekf_pose_position_string << z_ekf_pose_position;
  stat.addf("z Ekf Pose Position", z_ekf_pose_position_string.str().c_str());
}

/*

  La funzione xEkfPoseOrientationDiagostic implementa il rilevamento a soglia per la variabile x_ekf_pose_orientation creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void xEkfPoseOrientationDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (x_ekf_pose_orientation > x_ekf_pose_orientation_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_ekf_pose_orientation e' fuori soglia! Valore limite: %f", x_ekf_pose_orientation_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_ekf_pose_orientation: %f", x_ekf_pose_orientation);

  
  stat.add("Diagnostica Ekf Pose Orientation", "Valore");
  std::ostringstream x_ekf_pose_orientation_string;
  x_ekf_pose_orientation_string << x_ekf_pose_orientation;
  stat.addf("x Ekf Pose Orientation", x_ekf_pose_orientation_string.str().c_str());
}

/*

  La funzione yEkfPoseOrientationDiagostic implementa il rilevamento a soglia per la variabile y_ekf_pose_orientation creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void yEkfPoseOrientationDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (y_ekf_pose_orientation > y_ekf_pose_orientation_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_ekf_pose_orientation e' fuori soglia! Valore limite: %f", y_ekf_pose_orientation_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_ekf_pose_orientation: %f", y_ekf_pose_orientation);

  
  stat.add("Diagnostica Ekf Pose Orientation", "Valore");
  std::ostringstream y_ekf_pose_orientation_string;
  y_ekf_pose_orientation_string << y_ekf_pose_orientation;
  stat.addf("y Ekf Pose Orientation", y_ekf_pose_orientation_string.str().c_str());
}

/*

  La funzione zEkfPoseOrientationDiagostic implementa il rilevamento a soglia per la variabile z_ekf_pose_orientation creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void zEkfPoseOrientationDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (z_ekf_pose_orientation > z_ekf_pose_orientation_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_ekf_pose_orientation e' fuori soglia! Valore limite: %f", z_ekf_pose_orientation_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_ekf_pose_orientation: %f", z_ekf_pose_orientation);

  
  stat.add("Diagnostica Ekf Pose Orientation", "Valore");
  std::ostringstream z_ekf_pose_orientation_string;
  z_ekf_pose_orientation_string << z_ekf_pose_orientation;
  stat.addf("z Ekf Pose Orientation", z_ekf_pose_orientation_string.str().c_str());
}

/*

  La funzione wEkfPoseOrientationDiagostic implementa il rilevamento a soglia per la variabile w_ekf_pose_orientation creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void wEkfPoseOrientationDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (w_ekf_pose_orientation > w_ekf_pose_orientation_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore w_ekf_pose_orientation e' fuori soglia! Valore limite: %f", w_ekf_pose_orientation_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, w_ekf_pose_orientation: %f", w_ekf_pose_orientation);

  
  stat.add("Diagnostica Ekf Pose Orientation", "Valore");
  std::ostringstream w_ekf_pose_orientation_string;
  w_ekf_pose_orientation_string << w_ekf_pose_orientation;
  stat.addf("w Ekf Pose Orientation", w_ekf_pose_orientation_string.str().c_str());
}

/*

  La funzione xEkfTwistLinearDiagostic implementa il rilevamento a soglia per la variabile x_ekf_twist_linear creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void xEkfTwistLinearDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (x_ekf_twist_linear > x_ekf_twist_linear_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_ekf_twist_linear e' fuori soglia! Valore limite: %f", x_ekf_twist_linear_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_ekf_twist_linear: %f", x_ekf_twist_linear);

  
  stat.add("Diagnostica Ekf Twist Linear", "Valore");
  std::ostringstream x_ekf_twist_linear_string;
  x_ekf_twist_linear_string << x_ekf_twist_linear;
  stat.addf("x Ekf Twist Linear", x_ekf_twist_linear_string.str().c_str());
}

/*

  La funzione yEkfTwistLinearDiagostic implementa il rilevamento a soglia per la variabile y_ekf_twist_linear creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void yEkfTwistLinearDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (y_ekf_twist_linear > y_ekf_twist_linear_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_ekf_twist_linear e' fuori soglia! Valore limite: %f", y_ekf_twist_linear_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_ekf_twist_linear: %f", y_ekf_twist_linear);

  
  stat.add("Diagnostica Ekf Twist Linear", "Valore");
  std::ostringstream y_ekf_twist_linear_string;
  y_ekf_twist_linear_string << y_ekf_twist_linear;
  stat.addf("y Ekf Twist Linear", y_ekf_twist_linear_string.str().c_str());
}

/*

  La funzione zEkfTwistLinearDiagostic implementa il rilevamento a soglia per la variabile z_ekf_twist_linear creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void zEkfTwistLinearDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (z_ekf_twist_linear > y_ekf_twist_linear_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_ekf_twist_linear e' fuori soglia! Valore limite: %f", z_ekf_twist_linear_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_ekf_twist_linear: %f", z_ekf_twist_linear);

  
  stat.add("Diagnostica Ekf Twist Linear", "Valore");
  std::ostringstream z_ekf_twist_linear_string;
  z_ekf_twist_linear_string << z_ekf_twist_linear;
  stat.addf("z Ekf Twist Linear", z_ekf_twist_linear_string.str().c_str());
}

/*

  La funzione xEkfTwistAngularDiagostic implementa il rilevamento a soglia per la variabile x_ekf_twist_angular creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void xEkfTwistAngularDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (x_ekf_twist_angular > x_ekf_twist_angular_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_ekf_twist_angular e' fuori soglia! Valore limite: %f", x_ekf_twist_angular_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_ekf_twist_angular: %f", x_ekf_twist_angular);

  
  stat.add("Diagnostica Ekf Twist Angular", "Valore");
  std::ostringstream x_ekf_twist_angular_string;
  x_ekf_twist_angular_string << x_ekf_twist_angular;
  stat.addf("x Ekf Twist Angular", x_ekf_twist_angular_string.str().c_str());
}

/*

  La funzione yEkfTwistAngularDiagostic implementa il rilevamento a soglia per la variabile y_ekf_twist_angular creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void yEkfTwistAngularDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (y_ekf_twist_angular > y_ekf_twist_angular_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_ekf_twist_angular e' fuori soglia! Valore limite: %f", y_ekf_twist_angular_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_ekf_twist_angular: %f", y_ekf_twist_angular);

  
  stat.add("Diagnostica Ekf Twist Angular", "Valore");
  std::ostringstream y_ekf_twist_angular_string;
  y_ekf_twist_angular_string << y_ekf_twist_angular;
  stat.addf("y Ekf Twist Angular", y_ekf_twist_angular_string.str().c_str());
}

/*

  La funzione zEkfTwistAngularDiagostic implementa il rilevamento a soglia per la variabile z_ekf_twist_angular creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void zEkfTwistAngularDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (z_ekf_twist_angular > z_ekf_twist_angular_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_ekf_twist_angular e' fuori soglia! Valore limite: %f", z_ekf_twist_angular_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_ekf_twist_angular: %f", z_ekf_twist_angular);

  
  stat.add("Diagnostica Ekf Twist Angular", "Valore");
  std::ostringstream z_ekf_twist_angular_string;
  z_ekf_twist_angular_string << z_ekf_twist_angular;
  stat.addf("z Ekf Twist Angular", z_ekf_twist_angular_string.str().c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_ekf_diagnostic");
  ros::NodeHandle ekf_nh;
  
  diagnostic_updater::Updater ekf_updater;
  ekf_updater.setHardwareID("odometry/ekf_local");

  /*
    
    Caricamento delle variabili globali con le soglie ottenute dai valori raccolti dal file YAML di configurazione.

  */

  ekf_nh.getParam("/ekf_params/ekf_pose_position/x_threshold", x_ekf_pose_position_threshold);
  ekf_nh.getParam("/ekf_params/ekf_pose_position/y_threshold", y_ekf_pose_position_threshold);
  ekf_nh.getParam("/ekf_params/ekf_pose_position/z_threshold", z_ekf_pose_position_threshold);

  ekf_nh.getParam("/ekf_params/ekf_pose_orientation/x_threshold", x_ekf_pose_orientation_threshold);
  ekf_nh.getParam("/ekf_params/ekf_pose_orientation/y_threshold", y_ekf_pose_orientation_threshold);
  ekf_nh.getParam("/ekf_params/ekf_pose_orientation/z_threshold", z_ekf_pose_orientation_threshold);
  ekf_nh.getParam("/ekf_params/ekf_pose_orientation/w_threshold", w_ekf_pose_orientation_threshold);

  ekf_nh.getParam("/ekf_params/ekf_twist_linear/x_threshold", x_ekf_twist_linear_threshold);
  ekf_nh.getParam("/ekf_params/ekf_twist_linear/y_threshold", y_ekf_twist_linear_threshold);
  ekf_nh.getParam("/ekf_params/ekf_twist_linear/z_threshold", z_ekf_twist_linear_threshold);

  ekf_nh.getParam("/ekf_params/ekf_twist_angular/x_threshold", x_ekf_twist_angular_threshold);
  ekf_nh.getParam("/ekf_params/ekf_twist_angular/y_threshold", y_ekf_twist_angular_threshold);
  ekf_nh.getParam("/ekf_params/ekf_twist_angular/z_threshold", z_ekf_twist_angular_threshold);
    
  /*

    Il metodo subscribe permette di sottoscriversi al topic /odometry/ekf_local per estrarne i messaggi. Il metodo richiama la funzione imuCallback.

  */

  ros::Subscriber sub = ekf_nh.subscribe("odometry/ekf_local", 1000, ekfPosePositionCallback);
  
  /*
  
    Il metodo add permette la creazione del messaggio diagnostico invocando la funzione che effettuerà il rilevamento a soglia.

  */
 
  ekf_updater.add("Funzione di diagnostica della x ekf pose position", xEkfPosePositionDiagostic);
  ekf_updater.add("Funzione di diagnostica della y ekf pose position", yEkfPosePositionDiagostic);
  ekf_updater.add("Funzione di diagnostica della z ekf pose position", zEkfPosePositionDiagostic);

  ekf_updater.add("Funzione di diagnostica della x ekf pose orientation", xEkfPoseOrientationDiagostic);
  ekf_updater.add("Funzione di diagnostica della y ekf pose orientation", yEkfPoseOrientationDiagostic);
  ekf_updater.add("Funzione di diagnostica della z ekf pose orientation", zEkfPoseOrientationDiagostic);
  ekf_updater.add("Funzione di diagnostica della w ekf pose orientation", wEkfPoseOrientationDiagostic);

  ekf_updater.add("Funzione di diagnostica della x ekf twist linear", xEkfTwistLinearDiagostic);
  ekf_updater.add("Funzione di diagnostica della y ekf twist linear", yEkfTwistLinearDiagostic);
  ekf_updater.add("Funzione di diagnostica della z ekf twist linear", zEkfTwistLinearDiagostic);

  ekf_updater.add("Funzione di diagnostica della x ekf twist angular", xEkfTwistAngularDiagostic);
  ekf_updater.add("Funzione di diagnostica della y ekf twist angular", yEkfTwistAngularDiagostic);
  ekf_updater.add("Funzione di diagnostica della z ekf twist angular", zEkfTwistAngularDiagostic);
  

  while (ekf_nh.ok())
  {
    std_msgs::Bool msg;
    ros::Duration(0.1).sleep();
    ros::spinOnce();
   
    msg.data = false;
    
    ekf_updater.update();
  }

  return 0; 
}