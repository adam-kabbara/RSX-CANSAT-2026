#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

static constexpr uint8_t NUM_STATE = 18;

// ---------- Structs ----------
struct IMUData {
	float acc[3];
	float gyro[3];
	float mag[3];
	bool acc_valid;
	bool gyro_valid;
	bool mag_valid;
};

struct BaroData {
	float z;
	bool valid;
};

struct GPSData {
	float pos[3];
	float vel[3];
	bool valid;
};

struct MagCallibration {
	float W[3][3];	// Soft-iron correction
	float H[3];		// HArd-iron correction
	bool callibrated;
};

struct KF_Noise {
	// Process noise (Q)
	float q_pos;		// position			[ m^2/s ]
	float q_vel;		// velocity			[ (m/s)^2/s ]
	float q_ori;		// orientation		[ rad^2/s ]
	float q_biasGyro;	// Bias of gyro		[ (rad/s)^2/s ]
	float q_biasAcc;	// Bias of accelerometer	[(m/s^2)^2/s]
	float q_biasMag;	// Bias of magnetometer		[ T^2/s ]

	// IMU input noise
	float sig_gyro;		// Gyro white noise 	[ rad/s / sqrt(Hz) ]
	float sig_acc;		// Accel. white noise	[ m/s^2 / sqrt(Hz) ]

	// Sensor noise (R)
	float r_gpsPos;		// GPS position		[ m^2 ]
	float r_gpsVel;		// GPS velocity		[ (m/s)^2 ]
	float r_baro;		// Barometer		[ m^2 ]
	float r_mag;		// Magnetometer		[ T^2 ]
};

struct KF_State {
	float x[NUM_STATE];				// State vector
	float S[NUM_STATE][NUM_STATE];	// Covariance matrix
	float dt;
};


// =============================
// 	       Kalman Filter
// =============================
class KalmanFilter {
	private:
		KF_Statae state_;
		KF_Noise noise_;
		bool isInitialized_;

	public:
		// ====== Constructors ======
		KalmanFilter() : state_{0}, noise_{0}, isInitialized_{false} {}

		// ====== Public Functions ======
		// ---- Start/Stop -----
		bool init();
		void reset();

		void enableIMU(bool enable);
		void enableAcc(bool enable);
		void enableGyro(bool enable);
		void enableMag(bool enable);
		void enableBaro(bool enable);
		void enableGPS(bool enable);

		// ---- Process -----
		void update();		// idk if sensor data should be captured in KF or in main loop
		void updateIMU();
		void updateGPS();
		void updateBaro();

		// ----- Getters -----
		KF_State getState();
		KF_Noise getNoise();
		MagCallibration  getMagCallibration();

		// ----- Setters -----
		void setMagCallibration(float *W, float *H);
};


#endif






