#include "main.h"

// SLOT //
int slot = 0;
std::string slotName = "Default Slot";

// LOGGER //
/**
 * @class Logger
 * @brief A class for logging messages with different levels of severity.
 */
class Logger {
	// Log Buffer (list of strings)
	std::vector<std::string> logBuffer;
	
	// Screen Selection
	bool showSelectionScreen = false;
	
	// Distance Sensor (list of 5 distances)
	int left_distances[5] = {};
	int right_distances[5] = {};

public:
	/**
	 * @brief Logs an error message with the specified subsystem and message.
	 * @param subsystem The subsystem associated with the error.
	 * @param message The error message to be logged.
	 */
	void error(std::string subsystem, std::string message) {
		// RED COLOR

		// Add to log buffer (w/ color)
		logBuffer.push_back("#ef2929 [ERROR] - [" + subsystem + "]: " + message + "#");

		// Print to console
		printf("[ERROR]: %s\n", message.c_str());

		// Update screen
		updateScreen();
	}

	/**
	 * @brief Logs a warning message with the specified subsystem and message.
	 * @param subsystem The subsystem associated with the warning.
	 * @param message The warning message to be logged.
	 */
	void warning(std::string subsystem, std::string message) {
		// Add to log buffer
		logBuffer.push_back("#fce94f [WARN] - [" + subsystem + "]: " + message + "#");

		// Print to console
		printf("[WARNING]: %s\n", message.c_str());

		// Update screen
		updateScreen();
	}

	/**
	 * @brief Logs an informational message with the specified subsystem and message.
	 * @param subsystem The subsystem associated with the information.
	 * @param message The informational message to be logged.
	 */
	void info(std::string subsystem, std::string message) {
		// Add to log buffer
		logBuffer.push_back("#ffffff [INFO] - [" + subsystem + "]: " + message + "#");

		// Print to console
		printf("[INFO]: %s\n", message.c_str());

		// Update screen
		updateScreen();
	}

	/**
	 * @brief Logs a debug message with the specified subsystem and message.
	 * @param subsystem The subsystem associated with the debug message.
	 * @param message The debug message to be logged.
	 */
	void debug(std::string subsystem, std::string message) {
		// Add to log buffer
		logBuffer.push_back("#d3d7cf [DEBUG] - [" + subsystem + "]: " + message + "#");

		// Print to console
		printf("[DEBUG]: %s\n", message.c_str());

		// Update screen
		updateScreen();
	}

	/**
	 * @brief Logs a verbose message with the specified subsystem and message.
	 * @param subsystem The subsystem associated with the verbose message.
	 * @param message The verbose message to be logged.
	 */
	void verbose(std::string subsystem, std::string message) {
		// Add to log buffer
		logBuffer.push_back("#555753 [VERBOSE] - [" + subsystem + "]: " + message + "#");

		// Print to console
		printf("[VERBOSE]: %s\n", message.c_str());

		// Update screen
		updateScreen();
	}

	/**
	 * @brief Enables the selection screen.
	 */
	void enableSelectionScreen() {
		showSelectionScreen = true;
		updateScreen();
	}

	/**
	 * @brief Disables the selection screen.
	 */
	void disableSelectionScreen() {
		showSelectionScreen = false;
		updateScreen();
	}
	
	/**
	 * @brief Sets the slot name for the selection screen.
	 */
	int getSlot() {
		return slot;
	}

	/**
	 * @brief Adds a new distance to the list of distances from the left distance sensor.
	 */
	void setDistanceLeft(int distance) {
		// Shift all distances
		for (int i = 4; i > 0; i--) {
			left_distances[i] = left_distances[i - 1];
		}
		// Set new distance
		left_distances[0] = distance;

		updateScreen();
	}

	/**
	 * @brief Adds a new distance to the list of distances from the right distance sensor.
	 */
	void setDistanceRight(int distance) {
		// Shift all distances
		for (int i = 4; i > 0; i--) {
			right_distances[i] = right_distances[i - 1];
		}
		// Set new distance
		right_distances[0] = distance;

		updateScreen();
	}


private:
	static lv_res_t blue1_callback(lv_obj_t * btn) {
		// Set slot
		slot = 1;
		slotName = "Blue 1";
		return LV_RES_OK;
	}

	static lv_res_t blue2_callback(lv_obj_t * btn) {
		// Set slot
		slot = 2;
		slotName = "Blue 2";
		return LV_RES_OK;
	}

	static lv_res_t red1_callback(lv_obj_t * btn) {
		// Set slot
		slot = 3;
		slotName = "Red 1";
		return LV_RES_OK;
	}

	static lv_res_t red2_callback(lv_obj_t * btn) {
		// Set slot
		slot = 4;
		slotName = "Red 2";
		return LV_RES_OK;
	}

	static lv_res_t skills_callback(lv_obj_t * btn) {
		// Set slot
		slot = 5;
		slotName = "Skills";
		return LV_RES_OK;
	}

	/**
	 * @brief Updates the screen with the latest log messages.
	 * @details This function removes any logs that are too old, clears the screen,
	 *          creates a label to display the log messages, and sets the text of the label.
	 */
	void updateScreen() {
		if (!showSelectionScreen) {
			// Remove any logs that are too old
			if (logBuffer.size() > 10) {
				logBuffer.erase(logBuffer.begin());
			}

			// Clear screen
			lv_obj_clean(lv_scr_act());
			lv_obj_t * label = lv_label_create(lv_scr_act(), NULL);

			// Create log string
			std::string logString = "";
			for (int i = 0; i < logBuffer.size(); i++) {
				logString += logBuffer[i] + "\n";
			}

			// Set text
			lv_label_set_text(label, logString.c_str());
			lv_label_set_recolor(label, true);
			lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);

			// Update screen
			lv_task_handler();
		} else {
			// Clear screen
			lv_obj_clean(lv_scr_act());

			// Get distance difference (left)
			int leftDistance = 0;
			for (int i = 0; i < 5; i++) {
				leftDistance += left_distances[i];
			}
			leftDistance /= 5;
			int leftDistanceDifference = leftDistance - OPTIMAL_DISTANCE_LEFT;

			// Get distance difference (right)
			int rightDistance = 0;
			for (int i = 0; i < 5; i++) {
				rightDistance += right_distances[i];
			}
			rightDistance /= 5;
			int rightDistanceDifference = rightDistance - OPTIMAL_DISTANCE_RIGHT;

			int distanceDifference = 0;
			int chosenDistance = 0;

			if (leftDistance < rightDistance) {
				distanceDifference = leftDistanceDifference;
				chosenDistance = leftDistance;
			} else {
				distanceDifference = rightDistanceDifference;
				chosenDistance = rightDistance;
			}

			// Create background
			lv_obj_t * background = lv_obj_create(lv_scr_act(), NULL);
			lv_obj_set_size(background, 480, 240);
			lv_obj_set_pos(background, 0, 0);

			// Create new style (w/ appropriate color)
			static lv_style_t style;
			lv_style_copy(&style, &lv_style_plain);
			// Too far (2mm+) - Red
			if (distanceDifference > 2) {
				style.body.main_color = LV_COLOR_HEX(0xFF0000);
				style.body.grad_color = LV_COLOR_HEX(0xFF0000);
				style.body.border.color = LV_COLOR_HEX(0xFF0000);
			// Too close (2mm+) - Yellow
			} else if (distanceDifference < -2) {
				style.body.main_color = LV_COLOR_HEX(0xFFFF00);
				style.body.grad_color = LV_COLOR_HEX(0xFFFF00);
				style.body.border.color = LV_COLOR_HEX(0xFFFF00);
			// Optimal
			} else {
				style.body.main_color = LV_COLOR_HEX(0x00FF00);
				style.body.grad_color = LV_COLOR_HEX(0x00FF00);
				style.body.border.color = LV_COLOR_HEX(0x00FF00);
			}
			lv_obj_set_style(background, &style);
			
			// Write distance
			lv_obj_t * label = lv_label_create(lv_scr_act(), NULL);
			lv_label_set_text(label, ("Distance: " + std::to_string(chosenDistance) + " mm").c_str());
			lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);

			// Write slot
			lv_obj_t * label2 = lv_label_create(lv_scr_act(), NULL);
			lv_label_set_text(label2, ("Slot: " + slotName).c_str());
			lv_obj_align(label2, NULL, LV_ALIGN_CENTER, 0, 50);

			// Create blue 1 button (top left, black border 1 px)
			lv_obj_t * blue1 = lv_btn_create(lv_scr_act(), NULL);
			lv_obj_set_size(blue1, 100, 50);
			lv_obj_set_pos(blue1, 0, 0);
			lv_obj_t * label3 = lv_label_create(blue1, NULL);
			lv_label_set_text(label3, "Blue 1");
			lv_obj_set_style(label3, &style);
			lv_btn_set_action(blue1, LV_BTN_ACTION_CLICK, blue1_callback);

			// Create blue 2 button (top right, black border 1 px)
			lv_obj_t * blue2 = lv_btn_create(lv_scr_act(), NULL);
			lv_obj_set_size(blue2, 100, 50);
			lv_obj_set_pos(blue2, 380, 0);
			lv_obj_t * label4 = lv_label_create(blue2, NULL);
			lv_label_set_text(label4, "Blue 2");
			lv_obj_set_style(label4, &style);
			lv_btn_set_action(blue2, LV_BTN_ACTION_CLICK, blue2_callback);

			// Create red 1 button (bottom left, black border 1 px)
			lv_obj_t * red1 = lv_btn_create(lv_scr_act(), NULL);
			lv_obj_set_size(red1, 100, 50);
			lv_obj_set_pos(red1, 0, 190);
			lv_obj_t * label5 = lv_label_create(red1, NULL);
			lv_label_set_text(label5, "Red 1");
			lv_obj_set_style(label5, &style);
			lv_btn_set_action(red1, LV_BTN_ACTION_CLICK, red1_callback);

			// Create red 2 button (bottom right, black border 1 px)
			lv_obj_t * red2 = lv_btn_create(lv_scr_act(), NULL);
			lv_obj_set_size(red2, 100, 50);
			lv_obj_set_pos(red2, 380, 190);
			lv_obj_t * label6 = lv_label_create(red2, NULL);
			lv_label_set_text(label6, "Red 2");
			lv_obj_set_style(label6, &style);
			lv_btn_set_action(red2, LV_BTN_ACTION_CLICK, red2_callback);

			// Create Skills button (bottom center, black border 1 px)
			lv_obj_t * skills = lv_btn_create(lv_scr_act(), NULL);
			lv_obj_set_size(skills, 100, 50);
			lv_obj_set_pos(skills, 190, 190);
			lv_obj_t * label7 = lv_label_create(skills, NULL);
			lv_label_set_text(label7, "Skills");
			lv_obj_set_style(label7, &style);
			lv_btn_set_action(skills, LV_BTN_ACTION_CLICK, skills_callback);				
		}

	}
};

// REPLAY // 
class Replay
{
private:
    // Time
	int time = 0;
	int actualTime = 0;
	int startTime = 0;

    // Logger
    Logger logger;

	// Ended
    bool ended = false;

    // File
    FILE *file;

    // Read line from file
    std::string readLine()
{
    std::string line;
    if (file)
    {
        char buffer[256];
        char* result = NULL;
        while ((result = fgets(buffer, sizeof(buffer), file)) != NULL)
        {
            line += buffer;
            if (line.back() == '\n')
            {
                line.pop_back();
                break;
            }
        }
        if (result == NULL && !feof(file)) {
            std::cerr << "Error reading from file\n";
        }
        if (feof(file)) {
            std::cout << "End of file reached\n";
        }
    }
    else {
        std::cerr << "File not open\n";
    }
    if (line.empty()) {
        std::cout << "Read an empty line\n";
    }
    return line;
}

public:
    // Constructor
	Replay() {}

    Replay(Logger logger)
    {
        // Open file
        std::string filePath = std::string("/usd/") + slotName + ".replay";
        file = fopen(filePath.c_str(), "r");

        // Set logger
        this->logger = logger;
		startTime = millis();

        // Log
        logger.info("REPLAY", "Replaying from " + filePath);
    }

    Replay(std::string fileName)
    {
        // Open file
        std::string filePath = std::string("/usd/") + fileName;
        file = fopen(filePath.c_str(), "r");

        // Set logger
        this->logger = logger;

        // Log
        logger.info("REPLAY", "Replaying from " + filePath);
    }

    // End replay
    int endReplay()
    {
		// Save & Log bytes written
		int bytesRead = ftell(file);
		logger.info("REPLAY", "Replay ended - Bytes read: " + std::to_string(bytesRead));
		
        // Close file
        fclose(file);

		// Calculate total time
		int totalTime = millis() - startTime;
		logger.info("REPLAY", "Total time: " + std::to_string(totalTime));

		// Return bytes written
		ended = true;
		return bytesRead;
    }

    // Get motor values (left, right, front, middle, back)
    std::tuple<int, int, int, int, int, int, int> getMotorValues(int currentTime)
    {
        // Get line
		try {
        	std::string line = readLine();

			// Split line
        std::string delimiter = ",";
        std::string timeValue = line.substr(0, line.find(delimiter));
        line.erase(0, line.find(delimiter) + delimiter.length());
        std::string leftMotorValue = line.substr(0, line.find(delimiter));
        line.erase(0, line.find(delimiter) + delimiter.length());
        std::string rightMotorValue = line.substr(0, line.find(delimiter));
        line.erase(0, line.find(delimiter) + delimiter.length());
        std::string frontMotorValue = line.substr(0, line.find(delimiter));
        line.erase(0, line.find(delimiter) + delimiter.length());
        std::string middleMotorValue = line.substr(0, line.find(delimiter));
        line.erase(0, line.find(delimiter) + delimiter.length());
        std::string backMotorValue = line.substr(0, line.find(delimiter));
        line.erase(0, line.find(delimiter) + delimiter.length());
		std::string blockerValue = line.substr(0, line.find(delimiter));
		line.erase(0, line.find(delimiter) + delimiter.length());
		std::string grabberValue = line.substr(0, line.find(delimiter));
		line.erase(0, line.find(delimiter) + delimiter.length());

		// Update time if it is 0
		if (time == 0) {
			time = std::stoi(timeValue);
		}
		if (actualTime == 0) {
			actualTime = currentTime;
		}

		// Calculate actual delta time
		int actualDeltaTime = currentTime - actualTime;

		// Calculate Expected Delta Time
		int expectedDeltaTime = std::stoi(timeValue) - time;

		// Calculate Difference
		int timeOffset = expectedDeltaTime - actualDeltaTime;

		// Wait for difference
		if (timeOffset > 0)
		{
			delay(timeOffset);
		}

        // Return motor values
        return std::make_tuple(std::stoi(leftMotorValue), std::stoi(rightMotorValue), std::stoi(frontMotorValue), std::stoi(middleMotorValue), std::stoi(backMotorValue), std::stoi(blockerValue), std::stoi(grabberValue));

		} catch (std::exception e) {
			this->endReplay();
		}

		return std::make_tuple(0, 0, 0, 0, 0, 0, 0);
    }

    bool isFinished()
    {
        return ended;
    }

};

// RECORDING //
class Recorder {
private:
    // Time
    int time = 0;
	int startTime = 0;

    // Logger
    Logger logger;

    // File
    FILE* file;

    // Write line to file
    void writeLine(std::string line) {
        // Write line to file
        fputs(line.c_str(), file);

		// Log ferror output (literally)
		if (ferror(file)) {
			logger.error("RECORD", "Error writing to file: " + std::to_string(ferror(file)));
		}
	}

public:
	Recorder() {}

    // Constructor
    Recorder(Logger logger) {
        // Open file
        std::string filePath = std::string("/usd/") + slotName + ".replay";
        file = fopen(filePath.c_str(), "w");

        // Set logger
        this->logger = logger;
		startTime = millis();
        
        // Log
        logger.info("RECORD", "Recording to " + filePath);
	}

    // End recording
    int endRecording() {
		// Save & Log bytes written
		int bytesWritten = ftell(file);
		logger.info("RECORD", "Recording ended - Bytes written: " + std::to_string(bytesWritten));
		
        // Close file
        fclose(file);

		// Calculate total time
		int totalTime = millis() - startTime;
		logger.info("RECORD", "Total time: " + std::to_string(totalTime));

		// Return bytes written
		return bytesWritten;
    }

    // Record Values
    void recordValues(int time, int leftPower, int rightPower, int frontPower, int middlePower, int backPower, bool blockerState, bool grabberState) {
        // Update time if it is 0
        if (this->time == 0) {
            this->time = time;
        }

        // Calculate delta time
        int deltaTime = time - this->time;

        // Update time
        this->time = time;

        // Write line to file
        std::string line = std::to_string(time) + "," + std::to_string(leftPower) + "," + std::to_string(rightPower) + "," + std::to_string(frontPower) + "," + std::to_string(middlePower) + "," + std::to_string(backPower) + "," + std::to_string(blockerState) + "," + std::to_string(grabberState) + "\n";
        writeLine(line);
	}
};

// Configure Devices - Brain
Controller master(E_CONTROLLER_MASTER);

// Configure Devices - Drivetrain
Motor leftMotor1(LEFT_MOTOR_PORT_1, LEFT_MOTOR_1_GEARSET, false);
Motor leftMotor2(LEFT_MOTOR_PORT_2, LEFT_MOTOR_2_GEARSET, false);

Motor rightMotor1(RIGHT_MOTOR_PORT_1, RIGHT_MOTOR_1_GEARSET, true);
Motor rightMotor2(RIGHT_MOTOR_PORT_2, RIGHT_MOTOR_2_GEARSET, true);

MotorGroup leftMotors({leftMotor1, leftMotor2});
MotorGroup rightMotors({rightMotor1, rightMotor2});

// Configure Devices - Rollers
Motor frontRoller1(FRONT_ROLLER_PORT_1, FRONT_ROLLER_1_GEARSET, false);
Motor frontRoller2(FRONT_ROLLER_PORT_2, FRONT_ROLLER_2_GEARSET, true);

Motor middlerRoller(MIDDLER_ROLLER_PORT, MIDDLER_ROLLER_GEARSET, true);

Motor backRoller(BACK_ROLLER_PORT, BACK_ROLLER_GEARSET, false);

MotorGroup frontRollers({frontRoller1, frontRoller2});

// Configure Devices - Solenoids
ADIDigitalOut blocker(BLOCKER_PORT);
ADIDigitalOut grabber(GRABBER_PORT);

// Configure Devices - Sensors
Distance leftDistanceSensor(LEFT_DISTANCE_SENSOR_PORT);
Distance rightDistanceSensor(RIGHT_DISTANCE_SENSOR_PORT);

// Configure Devices - Program
Logger logger = Logger();
Recorder recorder = Recorder();
Replay replay = Replay();
bool recording = false;
bool launchMode = false;
bool blockerState = false;
bool grabberState = false;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	// LVGL init
	lv_init();

	// Log Welcome Message
	logger.debug("MAIN", "LVGL Initialized");
	logger.info("MAIN", "Welcome to Vexinator v1.0.0!");

	// Check Prerequisites (SD Card)
	if (!usd::is_installed()) {
		logger.error("MAIN", "SD Card not present");
		return;
	}
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	// Log State Change
	logger.info("GAME", "Robot Disabled - Waiting for Enable");
	logger.enableSelectionScreen();

	// Loop distance sensor
	while (true) {
		// Get distance
		int leftDistance = leftDistanceSensor.get();
		int rightDistance = rightDistanceSensor.get();

		// Set distance
		logger.setDistanceLeft(leftDistance);
		logger.setDistanceRight(rightDistance);

		// Wait
		delay(100);
	}
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	// Log State Change
	logger.info("GAME", "Competition Initialized - Waiting for Enable");
	logger.enableSelectionScreen();

	// Loop distance sensor
	while (true) {
		// Get distance
		int leftDistance = leftDistanceSensor.get();
		int rightDistance = rightDistanceSensor.get();

		// Set distance
		logger.setDistanceLeft(leftDistance);
		logger.setDistanceRight(rightDistance);

		// Wait
		delay(100);
	}
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// Log State Change
	logger.info("GAME", "Autonomous Running");
	logger.disableSelectionScreen();

	// Load replay
	replay = Replay(logger);
	
	// Run replay
	while (!replay.isFinished()) {
		// Get motor values
		std::tuple<int, int, int, int, int, int, int> motorValues = replay.getMotorValues(millis());

		// Set motor values
		int max = 127;
		if (abs(std::get<0>(motorValues)) > max) {
			max = abs(std::get<0>(motorValues));
		}
		if (abs(std::get<1>(motorValues)) > max) {
			max = abs(std::get<1>(motorValues));
		}

		// Set motor values
		leftMotors.move((std::get<0>(motorValues) * DRIVE_SPEED / max * 127)/100);
		rightMotors.move((std::get<1>(motorValues) * DRIVE_SPEED / max * 127)/100);
		frontRollers.move(std::get<2>(motorValues));
		middlerRoller.move(std::get<3>(motorValues));
		backRoller.move(std::get<4>(motorValues));

		// Set solenoid values
		if (std::get<5>(motorValues)) {
			blocker.set_value(1);
		} else {
			blocker.set_value(0);
		}

		if (std::get<6>(motorValues)) {
			grabber.set_value(1);
		} else {
			grabber.set_value(0);
		}
	}

	// End replay
	replay.endReplay();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	// Log State Change
	logger.info("GAME", "Operator Control Running");
	logger.disableSelectionScreen();

	// Check recording status
	if (recording) {
		recording = false;
		recorder.endRecording();
		logger.warning("RECORD", "Recording Stopped due to Operator Control Restart");
	}

	while (true) {
		// DRIVETRAIN //
		// Get joystick values
		float xJoystick = master.get_analog(DRIVE_AXIS_CONTROLLER_X) * (TURN_SPEED / 100.0);
		float yJoystick = master.get_analog(DRIVE_AXIS_CONTROLLER_Y);

		// Calculate motor values
		float leftMotorValue = yJoystick + xJoystick;
		float rightMotorValue = yJoystick - xJoystick;

		// Max workaround (if joystick values add to more than 127)
		int max = 127;
		if (abs(leftMotorValue) > max) {
			max = abs(leftMotorValue);
		}
		if (abs(rightMotorValue) > max) {
			max = abs(rightMotorValue);
		}

		// Set motor values
		leftMotors.move((leftMotorValue * DRIVE_SPEED / max * 127)/100);
		rightMotors.move((rightMotorValue * DRIVE_SPEED / max * 127)/100);

		// SOLENOIDS // 
		if (master.get_digital_new_press(BLOCKER_TOGGLE_BUTTON)) {
			// Check current state
			if (blockerState) {
				logger.info("BLOCKER", "Retracted");
				blockerState = false;
				blocker.set_value(0);
			} else {
				logger.info("BLOCKER", "Deployed");
				blockerState = true;
				blocker.set_value(1);
			}
		}

		if (master.get_digital_new_press(GRABBER_TOGGLE_BUTTON)) {
			// Check current state
			if (grabberState) {
				logger.info("GRABBER", "Retracted");
				grabberState = false;
				grabber.set_value(0);
			} else {
				logger.info("GRABBER", "Deployed");
				grabberState = true;
				grabber.set_value(1);
			}
		}

		// ROLLERS //
		if (master.get_digital_new_press(ROLLER_FEEDER_TOGGLE_BUTTON)) {
			// Check launch mode
			if (launchMode) {
				launchMode = false;
				logger.info("ROLLER", "Launch Mode Disabled");
			} else {
				launchMode = true;
				logger.info("ROLLER", "Launch Mode Enabled");
			}
		}

		int frontRollerValue = 0;
		int middlerRollerValue = 0;
		int backRollerValue = 0;

		if (launchMode) {
			frontRollerValue = -127;
			middlerRollerValue = -127;
			backRollerValue = -127;
		} else {
			bool rollerFWDPressed = master.get_digital(ROLLER_FRONT_FWD_BUTTON);
			bool rollerREVPressed = master.get_digital(ROLLER_FRONT_REV_BUTTON);
			bool rollerLaunchFWDPressed = master.get_digital(ROLLER_LAUNCH_FWD_BUTTON);
			bool rollerLaunchREVPressed = master.get_digital(ROLLER_LAUNCH_REV_BUTTON);
			
			// Front Roller
			if (rollerREVPressed) {
				frontRollerValue = 127;
			} else if (rollerFWDPressed || rollerLaunchFWDPressed || rollerLaunchREVPressed) {
				frontRollerValue = -127;
			} else {
				frontRollerValue = 0;
			}

			// Middler Roller
			if (rollerLaunchFWDPressed) {
				middlerRollerValue = 127;
			} else if (rollerLaunchREVPressed) {
				middlerRollerValue = -127;
			} else {
				middlerRollerValue = 0;
			}

			// Back Roller
			if (rollerLaunchFWDPressed || rollerLaunchREVPressed) {
				backRollerValue = 127;
			} else {
				backRollerValue = 0;
			}
		}

		// Set motor values
		frontRollers.move(frontRollerValue);
		middlerRoller.move(middlerRollerValue);
		backRoller.move(backRollerValue);
		
		// RECORDING //
		if (recording) {
			recorder.recordValues(millis(), leftMotorValue, rightMotorValue, frontRollerValue, middlerRollerValue, backRollerValue, blockerState, grabberState);
		}
		
		if (master.get_digital_new_press(REPLAY_RECORD_BUTTON)) {
			// Check recording status
			if (!recording) {
				recording = true;
				recorder = Recorder(logger);
			} else {
				recording = false;
				recorder.endRecording();
			}
		}
		
		// wait
		delay(20);
	}
}
