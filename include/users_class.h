#include "main.h"
#include <string>

using namespace pros;
/**
* @class Users
* @brief class for organizing users
* @example
* setting order: name, slew acceleration, slew decelleration, exponent, scale
* factor. keybinds: high goal, medium goal, low goal, pneumatic button

* scale factor is relevant for arcade drive only
* Exponent is the input curve of the joystick to the output of the motors
* e.g. a higher exponenet will provide a greater exponential curve
* making the robot easier to control while trying to move slowy
* the slew max is the max acceleration per cycle
* the slew min is the maximum decelleration per cycle
* the control type (Arcade or Tank) allows the controlls of the robot to be
* either arcade or tank
* Keybinds:
* sf_high_goal (score for high goal): keybind that maps to high goal
* sf_medium_goal (score for medium goal): kebind that maps to score for middle
* goal
* sf_low_goal (score for low goal): keybind that scores for the low goal
* pn_button (pneumatics button): the button that maps to the intake funnel
* toggle
**/

/**
 * @example
 **/

class Users {
public:
  /**
   * @brief The different types of control schemes supported.
   */
  enum class ControlType { Arcade, Tank };

protected:
  std::string name;
  int SLEW_MAX;
  int SLEW_MIN;
  double SCALE_FACTOR;
  double EXPONENT_FOWARDS;
  double EXPONENT_TURN;
  ControlType control_type;
  // keybinds with default values
  controller_digital_e_t sf_high_goal =
      E_CONTROLLER_DIGITAL_R1; /// button for high goal conveyor
  controller_digital_e_t sf_medium_goal =
      E_CONTROLLER_DIGITAL_R2; /// button for low goal conveyor
  controller_digital_e_t sf_low_goal =
      E_CONTROLLER_DIGITAL_L1; //!< button for bottom goal conveyor
  controller_digital_e_t pn_button =
      E_CONTROLLER_DIGITAL_B; //!< button for pneumatic toggle

public:
  /**
   * Constructs a new Users object with the specified settings.
   *
   * \param name The driver's name.
   * \param slew_max The maximum acceleration per cycle.
   * \param slew_min The maximum deceleration per cycle.
   * \param exponent The exponent for the joystick curve.
   * \param scale_factor The scale factor for arcade drive.
   * \param control The control type (Arcade or Tank).
   * \param sf_high_goal Keybind for scoring a high goal.
   * \param sf_medium_goal Keybind for scoring a medium goal.
   * \param sf_low_goal Keybind for scoring a low goal.
   * \param pn_button Keybind for the intake funnel toggle.
   */

  // constructor
  Users(std::string name, int slew_max, int slew_min, double exponent_fw,
        double exponent_turn, double scale_factor, ControlType control,
        pros::controller_digital_e_t sf_high_goal = E_CONTROLLER_DIGITAL_R1,
        pros::controller_digital_e_t sf_medium_goal = E_CONTROLLER_DIGITAL_R2,
        pros::controller_digital_e_t sf_low_goal = E_CONTROLLER_DIGITAL_L1,
        pros::controller_digital_e_t pn_button = E_CONTROLLER_DIGITAL_B)
      : name(name), SLEW_MAX(slew_max), SLEW_MIN(slew_min),
        SCALE_FACTOR(scale_factor), EXPONENT_FOWARDS(exponent_fw),
        EXPONENT_TURN(exponent_turn), control_type(control),
        sf_high_goal(sf_high_goal), sf_medium_goal(sf_medium_goal),
        sf_low_goal(sf_low_goal), pn_button(pn_button) {}

  // setter
  void setDriverInfo(std::string newName, int slew_max, int slew_min,
                     double exponent, double scale_factor, ControlType control,
                     pros::controller_digital_e_t sf_high_goal,
                     pros::controller_digital_e_t sf_medium_goal,
                     pros::controller_digital_e_t sf_low_goal,
                     pros::controller_digital_e_t pn_button) {
    this->name = newName;
    this->SLEW_MAX = slew_max;
    this->SLEW_MIN = slew_min;
    this->EXPONENT_FOWARDS = exponent;
    this->SCALE_FACTOR = scale_factor;
    this->control_type = control;
    this->sf_high_goal = sf_high_goal;
    this->sf_medium_goal = sf_medium_goal;
    this->sf_low_goal = sf_low_goal;
    this->pn_button = pn_button;
  }

  // getters
  std::string getName() const { return name; }
  int getSlewMax() const { return SLEW_MAX; }
  int getSlewMin() const { return SLEW_MIN; }
  double getExponentFowards() const { return EXPONENT_FOWARDS; }
  double getExponentTurn() const { return EXPONENT_TURN; }
  double getScaleFactor() const { return SCALE_FACTOR; }
  ControlType getControlType() const { return control_type; }

  // getters for keybinds
  pros::controller_digital_e_t getSfHighGoal() const { return sf_high_goal; }
  pros::controller_digital_e_t getSfMediumGoal() const {
    return sf_medium_goal;
  }
  pros::controller_digital_e_t getSfBottomGoal() const { return sf_low_goal; }
  pros::controller_digital_e_t getPnButton() const { return pn_button; }

  static Users *currentUser;
};