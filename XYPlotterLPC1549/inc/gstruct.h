
/*
 * gcode.h
 *      Author: pavel
 */

#ifndef GSTRUCT_H_
#define GSTRUCT_H_

// structure for holding the G-code
typedef struct {

	char cmd_type[3];			/*command type of the G-code*/

	int x_pos;				/*goto x position value*/
	int y_pos;				/*goto y position value*/

	int pen_pos;				/*pen position(servo value)*/
	int pen_up;					/*pen up value*/
	int pen_dw;					/*pen down value*/

	int laserPower;					/*laser power (inverse)*/

	int x_dir;					/*stepper x-axis direction*/
	int y_dir;					/*stepper y-axis direction*/

	int speed;					/*speed of the stepper motor*/

	int plot_area_h;			/*height of the plotting area*/
	int plot_area_w;			/*width of the plotting area*/

	int abs;					/*coordinates absolute or relative(absolute: abs = 1)*/

}Gstruct;



#endif /* GSTRUCT_H_ */
