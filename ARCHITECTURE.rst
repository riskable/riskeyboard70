Riskey Firmware Architecture
============================
The Riskey firmware is based on Keyberon.  Keyberon was engineered for handling a traditional key switch matrix but the Riskey firmware was made for analog sensing.  Because of this, instead of using rows and columns to map particular key sensors to keyboard/encoder events we use analog multiplexers (aka 'muxes'; 74HC4067 ICs) and channels (i.e. mux/channel instead of row/column).

Configuration
--------------
The Riskey firmware includes a ``Config.toml`` file that (ultimately) generates the contents of ``config.rs`` (via an ``include!()`` macro) via ``build.rs`` (it reads the ``.toml`` file and generates ``userconfig.rs`` in the ``OUTPUT_DIR`` when you build the firmware).  All configuration items/settings will be added as ``pub const`` variables that are used in various locations throughout the code as ``config::<SOME_SETTING>``.

There's two types of configurable items:  Things that can be changed while the firmware is running (aka "runtime settings") and things that only matter at compile-time.  Regardless of how they're meant to be used, all settings are added to a ``config`` that is accessible via RTIC's ``LateResources`` struct (for convenient reference).  Having said that, for convenience (and no other reason) compile-time-only settings are usually used via the ``config::<SOME_SETTING>`` mechanism instead of pulling them out of ``c.resources.config.whatever.the_setting_in_question`` (because that's a lot more to type!).

If you wish to add a new configuration item you need to add it in three places:

* ``config_structs.rs``
* ``Config.toml``
* ``main.rs`` (where configuration gets initialized at the start of ``init()``

Layout of main.rs
-----------------
* Imports (``use`` lines)
* Type aliases
* The RTIC ``APP``...

APP Layout
~~~~~~~~~~
It's mostly a regular RTIC app with ``Resources`` at the top followed by an ``#[idle]`` function (to work around a bug related to ST-LINK2 flashing), and an ``#[init]`` function...

Init
~~~~
This is where our configuration structs get loaded into the ``config`` resource, stm32 boilerplate gets initialized, our timers (for USB polling and controlling the LEDs) are setup, all the pins are configured, display initilization, the initial analog values for all sensors are read (creates defaults), and the debugging channels get setup.

Tasks
-----
There's four tasks:
* ``usb_tx()``: USB polling send.
* ``usb_rx()``: USB polling receive.
* ``do_leds()``: Transforms the ``led_state`` and writes it out to the WS2812B LEDs.
* ``tick()``: Checks all sensors for key and rotary encoder events and executes their respective actions (finishes with a USB ``send_report()``).

Special Note About The Rotary encoder
-------------------------------------
Keyberon supports custom events but it works on the assumption that one key == one event.  Since our (analog hall effect) rotary encoder uses *two* sensors to generate clockwise, counterclockwise, and its own ``Press`` events we have to take some liberties (aka hacks):

* We use one encoder sensor (``config::ENCODER_CHANNEL1``) assignment in ``layers.rs`` to trigger a clockwise movement event and the other (``config::ENCODER_CHANNEL2``) to trigger counterclockwise event.  We're basically "faking it" by firing encoder events like they were keys assigned to custom Keyberon events.
* You can assign whatever actions you want for rotary encoder movements in ``layers.rs``.  Whatever counts as ``config::ENCODER_CHANNEL1`` will trigger when the rotary encoder is rotated clockwise and ``config::ENCODER_CHANNEL2`` will trigger when rotated counterclockwise.
