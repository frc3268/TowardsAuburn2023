package frc.robot.lib.units

/* Distance */

inline val Number.meters get() = this as Double
inline val Number.inches get() = this as Double * 0.0254

/* Angle */

inline val Number.deg get() = this as Double
inline val Number.rad get() = this as Double * (180 / Math.PI)

/* Time */

inline val Number.sec get() = this as Double