# Documentation Standards

Always use Doxygen Javadoc style documentation for all functions, classes, and methods in C++ and Python.

## C++

### C++ General Rules

- Apply documentation to headers (.h) and implementation files (.cpp, .ino).
- Use `/** ... */` for C++ blocks.
- Use `constexpr` and `ALL_CAPS` for global constants.
- Use `camelCase` for global variables.

### Files

- Start with `@file`.
- Add a `@author` followed by an empty line.
- Add a `@brief` one line description followed by an empty line.
- Add a `@details` detailed description if necessary.
- Use imperative voice for detailed description.
- Always put an empty line after details.
- Add a `@version`
- Add a `@date`
- Add a `@copyright`
- Example:

  ```cpp
  /**
   * @file PropaneScale.ino
   * @author Gerald Manweiler
   * 
   * @brief Main application file for ESP32-based propane level scale using HX711 amplifier.
   * 
   * @details Implements serial command interface for calibration and weight reporting, 
   * manages HX711 interactions, and applies calibration factors to convert raw readings to weight in pounds.
   * 
   * @version 0.1
   * @date 2024-06-01
   * @copyright Copyright (c) 2024
   */
  ```

### Functions and Methods

- Start with a `@brief` one line description followed by an empty line.
- Add a `@details` detailed description if necessary.
- Use imperative voice for detailed description.
- Always put an empty line after details.
- Use `@param {type}[in][out] name description` for all parameters.
- Use `@return {type} description` for return values.
- Always put an empty line after return lines.
- Use `@throws {type} description`
- Use `camelCase` for function names.
- Example:

  ```cpp
  /**
  * @brief Calculates the area of a circle.
  *
  * @details Imperative voice detailed description.
  *
  * @param[in] {double} radius The radius of the circle.
  * @return {double} The calculated area.
  *
  * @throws {std::invalid_argument} If radius is negative.
  */
  double getArea(double radius);
  ```

### Enums

- Use `PascalCase` for enum type names and `ALL_CAPS` for enumerators
- Always start with `@enum` followed by an empty line.
- Always add a `@brief` one line description followed by an empty line.
- Add a `@details` detailed description if necessary.
- Use imperative voice for detailed description.
- Always put an empty line after details.
- Always document the the intent of each enumerator with a javadoc style inline comment `/**< */`
- Example:

  ```cpp
  /**
  * @enum EepromLoadInitStatus
  * 
  * @brief Enum representing status of EEPROM load or initialization.
  * 
  * @details Defines possible outcomes attempting loading float value from EEPROM.
  */
  enum EepromLoadInitStatus {
    EEPROM_VALUE_LOADED_VALID = 0,                            /**< Value successfully loaded from EEPROM, passed validation. */
    EEPROM_VALUE_INITIALIZED_AFTER_LOAD_FAIL = 1,             /**< Value failed loading from EEPROM, default initialized, save attempted. */
    EEPROM_VALUE_INITIALIZED_AFTER_INVALID_LOAD = 2           /**< Value loaded from EEPROM but failed validation, default initialized, save attempted. */
  };
  ```

## Python

### Python general Rules

- Use triple single quotes `''' ... '''` for Python.

### Defs

- Use `@return {type} name description` for all Python return values
- Explicitly include types in the `{type}` format even if type hints are present.
- Use `@exception {type} description`.
- Example:

  ```python
  def get_area(radius):
      '''
      @brief Calculates the area of a circle.

      @details A detailed description if necessary.

      @param {float} radius The radius of the circle.
      @return {float} area The calculated area.

      @exception ExceptionType Description of exceptions raised.
      '''

      area = 0.00
      area = 3.14 * radius ** 2
      return area
  ```
