// update the state vector with a mesurment
int fusion_pos(
double dt, // time since last measurement 
const Eigen::VectorXd& y, // measurment vector
const Eigen::VectorXd x0, // actual state vector
const Eigen::MatrixXd& C  // measurement matrix : C_lh for light house measurement and C_IMU for IMU measurement
);
// define the measurments matrix C_lh and C_IMU
int define_measurement_matrix_pos();
