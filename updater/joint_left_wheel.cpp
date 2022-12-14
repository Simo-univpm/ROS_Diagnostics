/*

  il codice implementa il nodo per il controllo dei valori di joint_left_wheel e del PID
  per il rilevamento a soglia per rilevare eventuali valori anomali.

*/
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <control_msgs/PidState.h>
#include <sstream>

/*

  Variabili globali per la memorizzazione dei valori attuali dei sensori e delle soglie.

*/

float left_output = 0.0;
float left_output_threshold = 0.0;
float left_p_error = 0.0;
float left_p_error_threshold = 0.0;
float left_i_error = 0.0;
float left_i_error_threshold = 0.0;
float left_d_error = 0.0;
float left_d_error_threshold = 0.0;
float left_error = 0.0;
float left_error_threshold = 0.0;
float left_error_dot = 0.0;
float left_error_dot_threshold = 0.0;

/*

  La LeftJointStateCallback permette di salvare nelle variabili globali i valori attuali dei sensori.

*/
void LeftJointCallback(const control_msgs::PidState::ConstPtr& msg)
{
  left_output = msg->output;
  left_p_error = msg->p_error;
  left_i_error = msg->i_error;
  left_d_error = msg->d_error;
  left_error = msg->error;
  left_error_dot = msg->error_dot;
}

/*
  La funzione LeftJointOutputDiagostic implementa il rilevamento a soglia per la variabile output creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/
void LeftJointOutputDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (left_output > left_output_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore e' fuori soglia! Soglia: %f.", left_output_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, left_output: %f", left_output);

  
  stat.add("Diagnostica left_wheel_joint", "Valore");
  std::ostringstream left_output_string;
  left_output_string << left_output;
  stat.addf("Left Wheel output", left_output_string.str().c_str());
}

/*

  La funzione LeftJointPErrorDiagostic implementa il rilevamento a soglia per la variabile p_error creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.

*/
void LeftJointPErrorDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (left_p_error > left_p_error_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore e' fuori soglia! Soglia: %f.", left_p_error_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, left_p_error: %f", left_p_error);

  
  stat.add("Diagnostica left_wheel_joint", "Valore");
  std::ostringstream left_p_error_string;
  left_p_error_string << left_p_error;
  stat.addf("Left Wheel p_error", left_p_error_string.str().c_str());
}

/*
  
  La funzione LeftJointIErrorDiagostic implementa il rilevamento a soglia per la variabile i_error creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/
void LeftJointIErrorDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (left_i_error > left_i_error_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore e' fuori soglia! Soglia: %f.", left_i_error_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, left_i_error: %f", left_i_error);

  
  stat.add("Diagnostica left_wheel_joint", "Valore");
  std::ostringstream left_i_error_string;
  left_i_error_string << left_i_error;
  stat.addf("Left Wheel i_error", left_i_error_string.str().c_str());
}

/*
  
  La funzione LeftJointDErrorDiagostic implementa il rilevamento a soglia per la variabile d_error creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/
void LeftJointDErrorDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (left_d_error > left_d_error_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore e' fuori soglia! Soglia: %f.", left_d_error_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, left_d_error: %f", left_d_error);

  
  stat.add("Diagnostica left_wheel_joint", "Valore");
  std::ostringstream left_d_error_string;
  left_d_error_string << left_d_error;
  stat.addf("Left Wheel d_error", left_d_error_string.str().c_str());
}

/*
  
  La funzione LeftJointErrorDiagostic implementa il rilevamento a soglia per la variabile error creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/
void LeftJointErrorDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (left_error > left_error_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore e' fuori soglia! Soglia: %f.", left_error_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, left_error: %f", left_error);

  
  stat.add("Diagnostica left_wheel_joint", "Valore");
  std::ostringstream left_error_string;
  left_error_string << left_error;
  stat.addf("Left Wheel error", left_error_string.str().c_str());
}

/*
  
  La funzione LeftJointErrorDotDiagostic implementa il rilevamento a soglia per la variabile error_dot creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/
void LeftJointErrorDotDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (left_error_dot > left_error_dot_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore e' fuori soglia! Soglia: %f.", left_error_dot_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, left_error_dot: %f", left_error_dot);

  
  stat.add("Diagnostica left_wheel_joint", "Valore");
  std::ostringstream left_error_dot_string;
  left_error_dot_string << left_error_dot;
  stat.addf("Left Wheel error_dot", left_error_dot_string.str().c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "left_wheel_joint_diagnostic");
  ros::NodeHandle nh;
  diagnostic_updater::Updater updater;
  updater.setHardwareID("left_wheel_joint/state");
  /*
    
    Caricamento delle variabili globali con le soglie ottenute dai valori raccolti dal file YAML di configurazione.
  
  */
  nh.getParam("/left_joint_wheel_params/left_joint_wheel_thresholds/output", left_output_threshold);
  nh.getParam("/left_joint_wheel_params/left_joint_wheel_thresholds/output", left_p_error_threshold);
  nh.getParam("/left_joint_wheel_params/left_joint_wheel_thresholds/output", left_i_error_threshold);
  nh.getParam("/left_joint_wheel_params/left_joint_wheel_thresholds/output", left_d_error_threshold);
  nh.getParam("/left_joint_wheel_params/left_joint_wheel_thresholds/output", left_error_threshold);
  nh.getParam("/left_joint_wheel_params/left_joint_wheel_thresholds/output", left_error_dot_threshold);
  
  /*

    Il metodo subscribe permette di sottoscriversi al topic /left_wheel_joint/state per estrarne i messaggi.
    Il metodo richiama le funzioni LeftJointOutputDiagostic, LeftJointPErrorDiagostic,
      LeftJointIErrorDiagostic, LeftJointDErrorDiagostic, LeftJointErrorDiagostic, LeftJointErrorDotDiagostic.
  
  */

  ros::Subscriber sub = nh.subscribe("/gazebo_ros_control/pid_gains/left_wheel_joint/state", 1000, LeftJointCallback);

  updater.add("Diagnostica Output Joint left wheel: ", LeftJointOutputDiagostic);
  updater.add("Diagnostica P Error Joint left wheel: ", LeftJointPErrorDiagostic);
  updater.add("Diagnostica I Error Joint left wheel: ", LeftJointIErrorDiagostic);
  updater.add("Diagnostica D Error Joint left wheel: ", LeftJointDErrorDiagostic);
  updater.add("Diagnostica Error Joint left wheel: ", LeftJointErrorDiagostic);
  updater.add("Diagnostica Error_dot Joint left wheel: ", LeftJointErrorDotDiagostic);


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