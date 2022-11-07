# Changelog

This is the Changelog of the ACME framework
This file should be kept up to date following [these guidelines](https://keepachangelog.com/en/1.0.0/)

## [Unreleased]

### Added

### Changed

### Fixed

## [v1.3] 2022-11-07

### Changed

- Remove frame convention in simple rudder polar definition (json file). By convention COMEFROM convention is used
- Remove direction convention from polar definition (equivalent polar definition for NED or NWU direction convention)
- Change "flow_incident_on_main_rudder_deg" key into "angle_of_attack_deg"
- Enable polar rudder definition without cn coefficients (cn coefficients set to zero in that case)

### Fixed 

- Desactivate flow slipstream correction without propeller rotation speed.

## [v1.2] 2022-04-12

### Added

- Straightening factor added to rudder param
- Log messages added to propeller rudder base
- Added interaction model in MMG prop-rudder model
- Logging messages from hermes

### Changed

- Refactoring performance json file

### Fixed

- Fujii rudder model 
- Propeller rudder distance from propeller to rudder

## [v1.1] 2021-11-10

### Added 

- Brix parameters for rudder model
- Fujii parameters for rudder model
- Documentation update

### Changed

- Use mathutils v1.8
- Update validation

### Fixed

- Lift coefficient
- Propeller rudder computation


## [v1.0] 2021-08-13

### Added

- First version with all files