//my_cli_bindings.c

#include "embedded_cli.h"
#include "my_cli_bindings.h"
#include "../cope_hal/cope_leds.h"
#include "../cope_hal/cope_ls_data_port.h"
#include <stdlib.h>
extern EmbeddedCli * cli;
extern cope_led_struct my_ui_leds[4];

void * nullPtr = NULL;

// struct CliCommandBinding {
//     /**
//      * Name of command to bind. Should not be NULL.
//      */
//     const char *name;

//     /**
//      * Help string that will be displayed when "help <cmd>" is executed.
//      * Can have multiple lines separated with "\r\n"
//      * Can be NULL if no help is provided.
//      */
//     const char *help;

//     /**
//      * Flag to perform tokenization before calling binding function.
//      */
//     bool tokenizeArgs;

//     /**
//      * Pointer to any specific app context that is required for this binding.
//      * It will be provided in binding callback.
//      */
//     void *context;

//     /**
//      * Binding function for when command is received.
//      * If null, default callback (onCommand) will be called.
//      * @param cli - pointer to cli that is calling this binding
//      * @param args - string of args (if tokenizeArgs is false) or tokens otherwise
//      * @param context
//      */
//     void (*binding)(EmbeddedCli *cli, char *args, void *context);
// };
void init_cli_bindings(void) {
	

	CliCommandBinding temp_bind;

	temp_bind.name = "rec-led-on";
	temp_bind.help = "Turns the REC LED on";
	temp_bind.tokenizeArgs = false;
	temp_bind.context = nullPtr;
	temp_bind.binding = cli_rec_led_on;

	embeddedCliAddBinding( cli, temp_bind);


	temp_bind.name = "rec-led-off";
	temp_bind.help = "Turns the REC LED off";
	temp_bind.tokenizeArgs = false;
	temp_bind.context = nullPtr;
	temp_bind.binding = cli_rec_led_off;

	embeddedCliAddBinding( cli, temp_bind);

	temp_bind.name = "play-led-on";
	temp_bind.help = "Turns the PLAY LED on";
	temp_bind.tokenizeArgs = false;
	temp_bind.context = nullPtr;
	temp_bind.binding = cli_play_led_on;

	embeddedCliAddBinding( cli, temp_bind);


	temp_bind.name = "play-led-off";
	temp_bind.help = "Turns the REC PLAY off";
	temp_bind.tokenizeArgs = false;
	temp_bind.context = nullPtr;
	temp_bind.binding = cli_play_led_off;

	embeddedCliAddBinding( cli, temp_bind);

	temp_bind.name = "fbk-led-on";
	temp_bind.help = "Turns the fbk LED on";
	temp_bind.tokenizeArgs = false;
	temp_bind.context = nullPtr;
	temp_bind.binding = cli_fbk_led_on;

	embeddedCliAddBinding( cli, temp_bind);


	temp_bind.name = "fbk-led-off";
	temp_bind.help = "Turns the FBK LED off";
	temp_bind.tokenizeArgs = false;
	temp_bind.context = nullPtr;
	temp_bind.binding = cli_fbk_led_off;

	embeddedCliAddBinding( cli, temp_bind);

	temp_bind.name = "beat-led-on";
	temp_bind.help = "Turns the BEAT LED on";
	temp_bind.tokenizeArgs = false;
	temp_bind.context = nullPtr;
	temp_bind.binding = cli_fbk_led_on;

	embeddedCliAddBinding( cli, temp_bind);


	temp_bind.name = "beat-led-off";
	temp_bind.help = "Turns the BEAT LED off";
	temp_bind.tokenizeArgs = false;
	temp_bind.context = nullPtr;
	temp_bind.binding = cli_beat_led_off;

	embeddedCliAddBinding( cli, temp_bind);

	temp_bind.name = "led-set";
	// temp_bind.help = "Set arbitrary led to arbitrary brightness. Led names: rec, play, fbk, beat or all. Brightness 0-1.";
	temp_bind.help = "Set arbitrary led to arbitrary brightness.";
	temp_bind.tokenizeArgs = true;
	temp_bind.context = nullPtr;
	temp_bind.binding = cli_led_set;

	embeddedCliAddBinding( cli, temp_bind);

}

//8 brute force led bindings, for on or off control of each led

void cli_rec_led_on(EmbeddedCli *cli, char *args, void *context) {

	my_ui_leds[0].target_brightness = 1;
	
}

void cli_rec_led_off(EmbeddedCli *cli, char *args, void *context) {

	my_ui_leds[0].target_brightness = 0;
	
}

void cli_play_led_on(EmbeddedCli *cli, char *args, void *context) {

	my_ui_leds[1].target_brightness = 1;
	
}

void cli_play_led_off(EmbeddedCli *cli, char *args, void *context) {

	my_ui_leds[1].target_brightness = 0;
	
}

void cli_fbk_led_on(EmbeddedCli *cli, char *args, void *context) {

	my_ui_leds[2].target_brightness = 1;
	
}

void cli_fbk_led_off(EmbeddedCli *cli, char *args, void *context) {

	my_ui_leds[2].target_brightness = 0;
	
}

void cli_beat_led_on(EmbeddedCli *cli, char *args, void *context) {

	my_ui_leds[3].target_brightness = 1;
	
}

void cli_beat_led_off(EmbeddedCli *cli, char *args, void *context) {

	my_ui_leds[3].target_brightness = 0;
	
}

// arbitrary led binding
void cli_led_set(EmbeddedCli *cli, char *args, void *context) 
{

// should take in a name of a led and a brightness value
// name can be "rec", "play", "fbk", "beat" or "all"

	const char * led_name = embeddedCliGetToken(args, 1);
	const float led_brightness = atof(embeddedCliGetToken(args, 2));
	
	switch (led_name[0]) {
		case 'r':
			my_ui_leds[0].target_brightness = led_brightness;
			break;
		case 'p':
			my_ui_leds[1].target_brightness = led_brightness;
			break;
		case 'f':
			my_ui_leds[2].target_brightness = led_brightness;
			break;
		case 'b':
			my_ui_leds[3].target_brightness = led_brightness;
			break;
		case 'a':
			my_ui_leds[0].target_brightness = led_brightness;
			my_ui_leds[1].target_brightness = led_brightness;
			my_ui_leds[2].target_brightness = led_brightness;
			my_ui_leds[3].target_brightness = led_brightness;
			break;
		default:
			break;
	}
}


extern cope_data_packet_struct * my_tx_data_packet ;
void cli_print_cope_port_data(EmbeddedCli *cli, char *args, void *context)
{
	const char * format = embeddedCliGetToken(args, 1);

	switch (format[0]) {
		case 'j':
			//print json packed
			cope_ls_data_port_tx_dma_init( my_tx_data_packet->buffer, my_tx_data_packet->index );
			break;
		case 'r':
			//print readable json
			break;
		default:
			break;
	}

}


