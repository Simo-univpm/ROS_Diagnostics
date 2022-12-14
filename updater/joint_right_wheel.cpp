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
float right_output = 0.0;
float right_output_threshold = 0.0;
float right_p_error = 0.0;
float right_p_error_threshold = 0.0;
float right_i_error = 0.0;
float right_i_error_threshold = 0.0;
float right_d_error = 0.0;
float right_d_error_threshold = 0.0;
float right_error = 0.0;
float right_error_threshold = 0.0;
float right_error_dot = 0.0;
float right_error_dot_threshold = 0.0;

/*

  La RightJointStateCallback permette di salvare nelle variabili globali i valori attuali dei sensori.

*/
void RightJointCallback(const control_msgs::PidState::ConstPtr& msg)
{
  right_output = msg->output;
  right_p_error = msg->p_error;
  right_i_error = msg->i_error;
  right_d_error = msg->d_error;
  right_error = msg->error;
  right_error_dot = msg->error_dot;
}

/*
  La funzione RightJointOutputDiagostic implementa il rilevamento a soglia per la variabile output creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
  
*/
void RightJointOutputDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (right_output > right_output_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore e' fuori soglia! Soglia: %f.", right_output_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, output: %f", right_output);

  
  stat.add("Diagnostica right_wheel_joint", "Valore");
  std::ostringstream right_output_string;
  right_output_string << right_output;
  stat.addf("Right Wheel output", right_output_string.str().c_str());
}

/*
  la funzione RightJointPErrorDiagostic implementa il rilevamento a soglia per la variabile p_error creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
*/
void RightJointPErrorDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (right_p_error > right_p_error_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore e' fuori soglia! Soglia: %f.", right_p_error_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, p_error: %f", right_p_error);

  
  stat.add("Diagnostica right_wheel_joint", "Valore");
  std::ostringstream right_p_error_string;
  right_p_error_string << right_p_error;
  stat.addf("Right Wheel p_error", right_p_error_string.str().c_str());
}

/*
  la funzione RightJointIErrorDiagostic implementa il rilevamento a soglia per la variabile i_error creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
*/
void RightJointIErrorDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (right_i_error > right_i_error_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore e' fuori soglia! Soglia: %f.", right_i_error_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, i_error: %f", right_i_error);

  
  stat.add("Diagnostica right_wheel_joint", "Valore");
  std::ostringstream right_i_error_string;
  right_i_error_string << right_i_error;
  stat.addf("Right Wheel i_error", right_i_error_string.str().c_str());
}

/*
  la funzione RightJointDErrorDiagostic implementa il rilevamento a soglia per la variabile d_error creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
*/
void RightJointDErrorDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (right_d_error > right_d_error_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore e' fuori soglia! Soglia: %f.", right_d_error_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, d_error: %f", right_d_error);

  
  stat.add("Diagnostica right_wheel_joint", "Valore");
  std::ostringstream right_d_error_string;
  right_d_error_string << right_d_error;
  stat.addf("Right Wheel d_error", right_d_error_string.str().c_str());
}

/*
  la funzione RightJointErrorDiagostic implementa il rilevamento a soglia per la variabile error creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
*/
void RightJointErrorDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (right_error > right_error_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore e' fuori soglia! Soglia: %f.", right_error_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, error: %f", right_error);

  
  stat.add("Diagnostica right_wheel_joint", "Valore");
  std::ostringstream right_error_string;
  right_error_string << right_error;
  stat.addf("Right Wheel error", right_error_string.str().c_str());
}

/*
  la funzione RightJointErrorDotDiagostic implementa il rilevamento a soglia per la variabile error_dot creando il messaggio diagnostico
  che viene pubblicato nel topic /diagnostics.
*/
void RightJointErrorDotDiagostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (right_error_dot > right_error_dot_threshold)
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Il valore e' fuori soglia! Soglia: %f.", right_error_dot_threshold);
  else
    
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Il valore rientra nella norma, error_dot: %f", right_error_dot);

  
  stat.add("Diagnostica right_wheel_joint", "Valore");
  std::ostringstream right_error_dot_string;
  right_error_dot_string << right_error_dot;
  stat.addf("Right Wheel error_dot", right_error_dot_string.str().c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "right_wheel_joint_diagnostic");
  ros::NodeHandle nh;
  diagnostic_updater::Updater updater;
  updater.setHardwareID("right_wheel_joint/state");
  /*

    Caricamento delle variabili globali con le soglie ottenute dai valori raccolti dal file YAML di configurazione.

  */
  nh.getParam("/right_joint_wheel_params/right_joint_wheel_thresholds/output", right_output_threshold);
  nh.getParam("/right_joint_wheel_params/right_joint_wheel_thresholds/p_error", right_p_error_threshold);
  nh.getParam("/right_joint_wheel_params/right_joint_wheel_thresholds/i_error", right_i_error_threshold);
  nh.getParam("/right_joint_wheel_params/right_joint_wheel_thresholds/d_error", right_d_error_threshold);
  nh.getParam("/right_joint_wheel_params/right_joint_wheel_thresholds/error", right_error_threshold);
  nh.getParam("/right_joint_wheel_params/right_joint_wheel_thresholds/error_dot", right_error_dot_threshold);

  /*

    Il metodo subscribe permette di sottoscriversi al topic /right_wheel_joint/state per estrarne i messaggi.
    Il metodo richiama le funzioni RightJointOutputDiagostic, RightJointPErrorDiagostic, RightJointIErrorDiagostic,
      RightJointDErrorDiagostic, RightJointErrorDiagostic, RightJointErrorDotDiagostic.

  */
  ros::Subscriber sub = nh.subscribe("/gazebo_ros_control/pid_gains/right_wheel_joint/state", 1000, RightJointCallback);

  updater.add("Diagnostica Output Joint Right Wheel: ", RightJointOutputDiagostic);
  updater.add("Diagnostica P error Joint Right Wheel: ", RightJointPErrorDiagostic);
  updater.add("Diagnostica I error Joint Right Wheel: ", RightJointIErrorDiagostic);
  updater.add("Diagnostica D error Joint Right Wheel: ", RightJointDErrorDiagostic);
  updater.add("Diagnostica error Joint Right Wheel: ", RightJointErrorDiagostic);
  updater.add("Diagnostica error_dot Joint Right Wheel: ", RightJointErrorDotDiagostic);

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