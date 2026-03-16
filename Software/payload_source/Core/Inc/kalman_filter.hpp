#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

static constexpr uint8_t NUM_STATE = 15;

// ---------- Structs ----------
struct IMUData {
	float acc[3];
	float gyro[3];
	float mag[3];
	bool valid;
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

struct KF_Noise {
	float q_pos
};

struct KF_State {
	float x[NUM_STATE];				// State vector
	float S[NUM_STATE][NUM_STATE];	// Covariance matrix
	bool initialized;
};


// =============================
// 	       Kalman Filter
// =============================
class KalmanFilter {
	private:

	public:
};


#endif
