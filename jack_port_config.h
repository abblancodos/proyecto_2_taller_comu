/**
 * jack_port_config.h
 *
 * JACK port name constants for easier connection management
 */

#ifndef JACK_PORT_CONFIG_H
#define JACK_PORT_CONFIG_H

namespace jack_ports {

// Application ports (created by jack_client.cpp)
// Client name: "dsp1" (see jack_client.cpp line 197)
// Port names: "output" and "input" (see jack_client.cpp lines 265, 268)
constexpr const char *APP_OUTPUT_L = "dsp1:output";
constexpr const char *APP_OUTPUT_R = "dsp1:output"; // Mono client, same port
constexpr const char *APP_INPUT_L = "dsp1:input";
constexpr const char *APP_INPUT_R = "dsp1:input"; // Mono client, same port

// System/Hardware ports (detected by jack_lsp)
constexpr const char *HEADSET_PLAYBACK_L =
    "RC30-026902, Gaming Headset [Nari Essential, Wireless, Receiver] Analog "
    "Stereo:playback_FL";
constexpr const char *HEADSET_PLAYBACK_R =
    "RC30-026902, Gaming Headset [Nari Essential, Wireless, Receiver] Analog "
    "Stereo:playback_FR";
constexpr const char *HEADSET_CAPTURE =
    "RC30-026902, Gaming Headset [Nari Essential, Wireless, Receiver] Analog "
    "Stereo:capture_MONO";

} // namespace jack_ports

#endif // JACK_PORT_CONFIG_H
