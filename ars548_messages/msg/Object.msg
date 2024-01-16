uint16 u_statussensor #tbd
uint32 u_id #Unique ID of object
uint16 u_age #Age of object
uint8 u_statusmeasurement #Object Status
uint8 u_statusmovement #Object Movement Status
uint16 u_position_invalidflags #tbd
uint8 u_position_reference #Reference point position
float32 u_position_x #X Position of reference point
float32 u_position_x_std #X Position Std
float32 u_position_y #Y Position of reference point
float32 u_position_y_std #Y Position Std
float32 u_position_z #Z Position of reference point
float32 u_position_z_std #Z Position Std
float32 u_position_covariancexy #Covariance X Y
float32 u_position_orientation #Object Orientation
float32 u_position_orientation_std #Orientation Std
uint8 u_existence_invalidflags #unused
float32 u_existence_probability #Probability of Existence
float32 u_existence_ppv #unused
uint8 u_classification_car #Car Classification Probability
uint8 u_classification_truck #Truck Classification Probability
uint8 u_classification_motorcycle #Motorcycle Classification Probability
uint8 u_classification_bicycle #Bicycle Classification Probability
uint8 u_classification_pedestrian #Pedestrian Classification Probability
uint8 u_classification_animal #Animal Classification Probability
uint8 u_classification_hazard #Hazard Classification Probability
uint8 u_classification_unknown #Unknown Classification Probability
uint8 u_classification_overdrivable #unused
uint8 u_classification_underdrivable #unused
uint8 u_dynamics_absvel_invalidflags #unused
float32 f_dynamics_absvel_x #X Absolute Velocity
float32 f_dynamics_absvel_x_std #X Absolute Velocity Std
float32 f_dynamics_absvel_y #Y Absolute Velocity
float32 f_dynamics_absvel_y_std #Y Absolute Velocity Std
float32 f_dynamics_absvel_covariancexy #Covariance Absolute Velocity X Y
uint8 u_dynamics_relvel_invalidflags #unused
float32 f_dynamics_relvel_x #X Relative Velocity
float32 f_dynamics_relvel_x_std #X Relative Velocity Std
float32 f_dynamics_relvel_y #Y Relative Velocity
float32 f_dynamics_relvel_y_std #Y Relative Velocity Std
float32 f_dynamics_relvel_covariancexy #Covariance Relative Velocity X Y
uint8 u_dynamics_absaccel_invalidflags #unused
float32 f_dynamics_absaccel_x #X Absolute Acceleration
float32 f_dynamics_absaccel_x_std #X Absolute Acceleration Std
float32 f_dynamics_absaccel_y #Y Absolute Acceleration
float32 f_dynamics_absaccel_y_std #Y Absolute Acceleration Std
float32 f_dynamics_absaccel_covariancexy #Covariance Absolute Acceleration X Y
uint8 u_dynamics_relaccel_invalidflags #unused
float32 f_dynamics_relaccel_x #X Relative Acceleration
float32 f_dynamics_relaccel_x_std #X Relative Acceleration Std
float32 f_dynamics_relaccel_y #Y Relative Acceleration
float32 f_dynamics_relaccel_y_std #Y Relative Acceleration Std
float32 f_dynamics_relaccel_covariancexy #Covariance Relative Acceleration X Y
uint8 u_dynamics_orientation_invalidflags #unused
float32 u_dynamics_orientation_rate_mean #Object Orientation Rate
float32 u_dynamics_orientation_rate_std #Orientation Rate Std
uint32 u_shape_length_status #(unused) Shape Length Status
uint8 u_shape_length_edge_invalidflags #(unused) Invalid Flags Shape Length 
float32 u_shape_length_edge_mean #Mean Shape Length
float32 u_shape_length_edge_std #(unused) Shape Length Std
uint32 u_shape_width_status #(unused) Shape Width Status 
uint8 u_shape_width_edge_invalidflags #(unused) Invalid Flags Shape Width
float32 u_shape_width_edge_mean #Mean Shape Width
float32 u_shape_width_edge_std #(unused) Shape Width Std 