package frc.robot.lib.units

/******************************************************************************/
/* Distance */

/** Meters */
inline val Number.meters get() =
    this as Double

/** Inches */
inline val Number.inches get() =
    this as Double * 0.0254

/******************************************************************************/
/* Angle */

/** Degrees */
inline val Number.deg get() =
    this as Double

/** Radians */
inline val Number.rad get() =
    this as Double * (180 / Math.PI)

/******************************************************************************/