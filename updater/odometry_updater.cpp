/*

  Il codice implementa il nodo updater odometrico per il diff drive controller della carrozzina. 
  Raccoglie i dati dal topic /diff_drive_controller/odom ed effettua un rilevamento a soglia per individuare potenziali guasti dovuti a valori anomali dei sensori.
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

float x_pose_position = 0.0;
float x_pose_position_threshold = 0.0;
float y_pose_position = 0.0;
float y_pose_position_threshold = 0.0;
float z_pose_position = 0.0;
float z_pose_position_threshold = 0.0;
float pose_covariance = 0.0;

float x_pose_orientation = 0.0;
float x_pose_orientation_threshold = 0.0;
float y_pose_orientation = 0.0;
float y_pose_orientation_threshold = 0.0;
float z_pose_orientation = 0.0;
float z_pose_orientation_threshold = 0.0;
float w_pose_orientation = 0.0;
float w_pose_orientation_threshold = 0.0;

float x_twist_linear = 0.0;
float x_twist_linear_threshold = 0.0;
float y_twist_linear = 0.0;
float y_twist_linear_threshold = 0.0;
float z_twist_linear = 0.0;
float z_twist_linear_threshold = 0.0;

float x_twist_angular = 0.0;
float x_twist_angular_threshold = 0.0;
float y_twist_angular = 0.0;
float y_twist_angular_threshold = 0.0;
float z_twist_angular = 0.0;
float z_twist_angular_threshold = 0.0;

/*

  La posePositionCallback permette di salvare nelle variabili globali i valori correnti dei sensori.

*/

void posePositionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  x_pose_position = msg->pose.pose.position.x;
  y_pose_position = msg->pose.pose.position.y;
  z_pose_position = msg->pose.pose.position.z;

  x_pose_orientation = msg->pose.pose.orientation.x;
  y_pose_orientation = msg->pose.pose.orientation.y;
  z_pose_orientation = msg->pose.pose.orientation.z;
  w_pose_orientation = msg->pose.pose.orientation.w;

  x_twist_linear = msg->twist.twist.linear.x;
  y_twist_linear = msg->twist.twist.linear.y;
  z_twist_linear = msg->twist.twist.linear.z;

  x_twist_angular = msg->twist.twist.angular.x;
  y_twist_angular = msg->twist.twist.angular.y;
  z_twist_angular = msg->twist.twist.angular.z;

}

/*

  La funzione xPosePositionDiagostic implementa il rilevamento a soglia per la variabile x_pose_position creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void xPosePositionDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (x_pose_position > x_pose_position_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_pose_position e' fuori soglia! Valore limite: %f", x_pose_position_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_pose_position: %f", x_pose_position);

  
  stat.add("Diagnostica Pose Position", "Valore");
  std::ostringstream x_pose_position_string;
  x_pose_position_string << x_pose_position;
  stat.addf("x Pose Position", x_pose_position_string.str().c_str());
}

/*

  La funzione yPosePositionDiagostic implementa il rilevamento a soglia per la variabile y_pose_position creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void yPosePositionDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (y_pose_position > y_pose_position_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_pose_position e' fuori soglia! Valore limite: %f", y_pose_position_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_pose_position: %f", y_pose_position);

  
  stat.add("Diagnostica Pose Position", "Valore");
  std::ostringstream y_pose_position_string;
  y_pose_position_string << y_pose_position;
  stat.addf("y Pose Position", y_pose_position_string.str().c_str());
}

/*

  La funzione zPosePositionDiagostic implementa il rilevamento a soglia per la variabile z_pose_position creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void zPosePositionDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (z_pose_position > z_pose_position_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_pose_position e' fuori soglia! Valore limite: %f", z_pose_position_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_pose_position: %f", z_pose_position);

  
  stat.add("Diagnostica Pose Position", "Valore");
  std::ostringstream z_pose_position_string;
  z_pose_position_string << z_pose_position;
  stat.addf("z Pose Position", z_pose_position_string.str().c_str());
}

/*

  La funzione xPoseOrientationDiagostic implementa il rilevamento a soglia per la variabile x_pose_orientation creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void xPoseOrientationDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (x_pose_orientation > x_pose_orientation_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_pose_orientation e' fuori soglia! Valore limite: %f", x_pose_orientation_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_pose_orientation: %f", x_pose_orientation);

  
  stat.add("Diagnostica Pose Orientation", "Valore");
  std::ostringstream x_pose_orientation_string;
  x_pose_orientation_string << x_pose_orientation;
  stat.addf("x pose orientation", x_pose_orientation_string.str().c_str());
}

/*

  La funzione yPoseOrientationDiagostic implementa il rilevamento a soglia per la variabile y_pose_orientation creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void yPoseOrientationDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (y_pose_orientation > y_pose_orientation_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_pose_orientation e' fuori soglia! Valore limite: %f", y_pose_orientation_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_pose_orientation: %f", y_pose_orientation);

  
  stat.add("Diagnostica Pose Orientation", "Valore");
  std::ostringstream y_pose_orientation_string;
  y_pose_orientation_string << y_pose_orientation;
  stat.addf("y pose orientation", y_pose_orientation_string.str().c_str());
}

/*

  La funzione zPoseOrientationDiagostic implementa il rilevamento a soglia per la variabile z_pose_orientation creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void zPoseOrientationDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (z_pose_orientation > z_pose_orientation_threshold)
  
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_pose_orientation e' fuori soglia! Valore limite: %f", z_pose_orientation_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_pose_orientation: %f", z_pose_orientation);

  
  stat.add("Diagnostica Pose Orientation", "Valore");
  std::ostringstream z_pose_orientation_string;
  z_pose_orientation_string << z_pose_orientation;
  stat.addf("z pose orientation", z_pose_orientation_string.str().c_str());
}

/*

  La funzione wPoseOrientationDiagostic implementa il rilevamento a soglia per la variabile w_pose_orientation creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void wPoseOrientationDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (w_pose_orientation > w_pose_orientation_threshold)
  
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore w_pose_orientation e' fuori soglia! Valore limite: %f", w_pose_orientation_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, w_pose_orientation: %f", w_pose_orientation);

  
  stat.add("Diagnostica Pose Orientation", "Valore");
  std::ostringstream w_pose_orientation_string;
  w_pose_orientation_string << w_pose_orientation;
  stat.addf("w pose orientation", w_pose_orientation_string.str().c_str());
}

/*

  La funzione xTwistLinearDiagnostic implementa il rilevamento a soglia per la variabile x_twist_linear creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void xTwistLinearDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (x_twist_linear > x_twist_linear_threshold)
  
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_twist_linear e' fuori soglia! Valore limite: %f", x_twist_linear_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_twist_linear: %f", x_twist_linear);
  
  stat.add("Diagnostica Twist Linear", "Valore");
  std::ostringstream x_twist_linear_string;
  x_twist_linear_string << x_twist_linear;
  stat.addf("x Twist Linear", x_twist_linear_string.str().c_str());
}

/*

  La funzione yTwistLinearDiagnostic implementa il rilevamento a soglia per la variabile y_twist_linear creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void yTwistLinearDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (y_twist_linear > y_twist_linear_threshold)
  
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_twist_linear e' fuori soglia! Valore limite: %f", y_twist_linear_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_twist_linear: %f", y_twist_linear);
  
  stat.add("Diagnostica Twist Linear", "Valore");
  std::ostringstream y_twist_linear_string;
  y_twist_linear_string << y_twist_linear;
  stat.addf("y Twist Linear", y_twist_linear_string.str().c_str());
}

/*

  La funzione zTwistLinearDiagnostic implementa il rilevamento a soglia per la variabile z_twist_linear creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void zTwistLinearDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (z_twist_linear > z_twist_linear_threshold)
  
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_twist_linear e' fuori soglia! Valore limite: %f", z_twist_linear_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_twist_linear: %f", z_twist_linear);
  
  stat.add("Diagnostica Twist Linear", "Valore");
  std::ostringstream z_twist_linear_string;
  z_twist_linear_string << z_twist_linear;
  stat.addf("z Twist Linear", z_twist_linear_string.str().c_str());
}

/*

  La funzione xTwistAngularDiagnostic implementa il rilevamento a soglia per la variabile x_twist_angular creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void xTwistAngularDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (x_twist_angular > x_twist_angular_threshold)
  
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore x_twist_angular e' fuori soglia! Valore limite: %f", x_twist_angular_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, x_twist_angular: %f", x_twist_angular);
  
  stat.add("Diagnostica Twist Angular", "Valore");
  std::ostringstream x_twist_angular_string;
  x_twist_angular_string << x_twist_angular;
  stat.addf("x Twist Angular", x_twist_angular_string.str().c_str());
}

/*

  La funzione yTwistAngularDiagnostic implementa il rilevamento a soglia per la variabile y_twist_angular creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void yTwistAngularDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (y_twist_angular > y_twist_angular_threshold)
  
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore y_twist_angular e' fuori soglia! Valore limite: %f", y_twist_angular_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, y_twist_angular: %f", y_twist_angular);
  
  stat.add("Diagnostica Twist Angular", "Valore");
  std::ostringstream y_twist_angular_string;
  y_twist_angular_string << y_twist_angular;
  stat.addf("y Twist Angular", y_twist_angular_string.str().c_str());
}

/*

  La funzione zTwistAngularDiagnostic implementa il rilevamento a soglia per la variabile z_twist_angular creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/

void zTwistAngularDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (z_twist_angular > z_twist_angular_threshold)
  
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore z_twist_angular e' fuori soglia! Valore limite: %f", z_twist_angular_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, z_twist_angular: %f", z_twist_angular);
  
  stat.add("Diagnostica Twist Angular", "Valore");
  std::ostringstream z_twist_angular_string;
  z_twist_angular_string << z_twist_angular;
  stat.addf("z Twist Angular", z_twist_angular_string.str().c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_diagnostic");
  ros::NodeHandle nh;
  
  diagnostic_updater::Updater updater;
  updater.setHardwareID("diff_drive_controller/odom");

  /*
    
    Caricamento delle variabili globali con le soglie ottenute dai valori raccolti dal file YAML di configurazione.

  */

  nh.getParam("/odometry_diff_drive_params/odom_diff_drive_pose_position/x_threshold", x_pose_position_threshold);
  nh.getParam("/odometry_diff_drive_params/odom_diff_drive_pose_position/y_threshold", y_pose_position_threshold);
  nh.getParam("/odometry_diff_drive_params/odom_diff_drive_pose_position/z_threshold", z_pose_position_threshold);

  nh.getParam("/odometry_diff_drive_params/odom_diff_drive_pose_orientation/x_threshold", x_pose_orientation_threshold);
  nh.getParam("/odometry_diff_drive_params/odom_diff_drive_pose_orientation/y_threshold", y_pose_orientation_threshold);
  nh.getParam("/odometry_diff_drive_params/odom_diff_drive_pose_orientation/z_threshold", z_pose_orientation_threshold);
  nh.getParam("/odometry_diff_drive_params/odom_diff_drive_pose_orientation/w_threshold", w_pose_orientation_threshold);

  nh.getParam("/odometry_diff_drive_params/odom_diff_drive_twist_linear/x_threshold", x_twist_linear_threshold);
  nh.getParam("/odometry_diff_drive_params/odom_diff_drive_twist_linear/y_threshold", y_twist_linear_threshold);
  nh.getParam("/odometry_diff_drive_params/odom_diff_drive_twist_linear/z_threshold", z_twist_linear_threshold);

  nh.getParam("/odometry_diff_drive_params/odom_diff_drive_twist_angular/x_threshold", x_twist_angular_threshold);
  nh.getParam("/odometry_diff_drive_params/odom_diff_drive_twist_angular/y_threshold", y_twist_angular_threshold);
  nh.getParam("/odometry_diff_drive_params/odom_diff_drive_twist_angular/z_threshold", z_twist_angular_threshold);

  /*

    Il metodo subscribe permette di sottoscriversi al topic /diff_drive_controller/odom per estrarne i messaggi. Il metodo richiama la funzione imuCallback.

  */

  ros::Subscriber sub = nh.subscribe("diff_drive_controller/odom", 1000, posePositionCallback);
  
  /*
  
    Il metodo add permette la creazione del messaggio diagnostico invocando la funzione che effettuerà il rilevamento a soglia.

  */
  updater.add("Funzione di diagnostica della x pose position", xPosePositionDiagostic);
  updater.add("Funzione di diagnostica della y pose position", yPosePositionDiagostic);
  updater.add("Funzione di diagnostica della z pose position", zPosePositionDiagostic);

  updater.add("Funzione di diagnostica della x pose orientation", xPoseOrientationDiagostic);
  updater.add("Funzione di diagnostica della y pose orientation", yPoseOrientationDiagostic);
  updater.add("Funzione di diagnostica della z pose orientation", zPoseOrientationDiagostic);
  updater.add("Funzione di diagnostica della w pose orientation", wPoseOrientationDiagostic);

  updater.add("Funzione di diagnostica della x twist linear", xTwistLinearDiagnostic);
  updater.add("Funzione di diagnostica della y twist linear", yTwistLinearDiagnostic);
  updater.add("Funzione di diagnostica della z twist linear", zTwistLinearDiagnostic);

  updater.add("Funzione di diagnostica della x twist angular", xTwistAngularDiagnostic);
  updater.add("Funzione di diagnostica della y twist angular", yTwistAngularDiagnostic);
  updater.add("Funzione di diagnostica della z twist angular", zTwistAngularDiagnostic);

  while (nh.ok())
  {
    std_msgs::Bool msg;
    ros::Duration(0.1).sleep();
    ros::spinOnce();
   
    msg.data = false;
    
    updater.update();
  }

  return 0; 
}