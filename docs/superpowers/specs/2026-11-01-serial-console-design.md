# Serial console for live tuning and bench debugging

Date: 2026-11-01. Approved in chat.

## Goal

Change core control values over USART6 while the firmware runs and see the effect at once, without reflashing. Make competition mode usable on the bench before a run sequence exists.

## Line handling

- A line whose first non-space character is `/` goes to the console parser. Every other line keeps today's single-letter grammar, including the immediate `S` stop.
- `S` inside a console line is ordinary text, so names such as `heading_tol` can't stop the robot.
- A console line ends at CR, LF or the UART idle event, so senders that add no line ending still work. Send a whole line at once: a terminal that sends each key separately needs local line editing turned on.

## Commands

| Command | Effect |
|---|---|
| `/` or `/help` | Console help |
| `/list [prefix]` | Name, value, range and default |
| `/set <name> <value>` | Validate, apply, print old and new |
| `/reset` | All parameters back to the compiled defaults; needs the robot stopped |
| `/plot <heading\|speed\|wheels> [hz]`, `/plot off` | `plot:a,b,c` lines, 1 to 100 Hz, default 50 |

Competition mode only:

| Command | Effect |
|---|---|
| `/task <code>` | Enter a task code as if it had been scanned |
| `/vision <mode> [color]` | Set the host's vision mode and colour |
| `/obs`, `/obs clear` | List lidar obstacles, or clear them while stopped |
| `/pose <x> <y> <heading>` | Set the field pose after moving the robot by hand; needs chassis READY |

Also in competition mode, `F/B/L/R/T/A` and `W` work instead of answering `BUSY`. Survey mode stays locked.

## Parameters

| Takes effect | Name | Default | Range | Decimals |
|---|---|---|---|---|
| Next tick | `heading_kp` | 2.0 | 0 to 20 | 3 |
| Next tick | `heading_ki` | 1.0 | 0 to 20 | 3 |
| Next tick | `heading_i_max` | 5.0 | 0 to 60 | 2 |
| Next tick | `heading_i_zone` | 8.0 | 0 to 180 | 2 |
| Next tick | `turn_max_speed` | 90 | 1 to 240 | 1 |
| Next tick | `distance_kp` | 1.5 | 0.1 to 10 | 3 |
| Next tick | `distance_tol` | 3.0 | 0.5 to 50 | 2 |
| Next tick | `heading_tol` | 0.1 | 0.01 to 10 | 3 |
| Next tick | `settle_ms` | 200 | 0 to 2000 | 0 |
| Next tick | `acc_linear` | 2000 | 100 to 5000 | 0 |
| Next tick | `acc_angular` | 720 | 30 to 3000 | 0 |
| Next path | `cmd_speed` | 100 | 10 to 400 | 1 |
| While stopped | `wheel_dia` | 77 | 40 to 150 | 2 |
| While stopped | `chassis_k` | 220.2 | 100 to 400 | 2 |
| While stopped | `path_deviation` | 5 | 0.5 to 50 | 2 |
| Next sample | `lidar_x`, `lidar_y` | 0 | -200 to 200 | 1 |
| Next sample | `lidar_yaw` | 0 | -180 to 180 | 2 |
| Next sample | `lidar_min` | 200 | 50 to 500 | 0 |
| Next sample | `lidar_max` | 1800 | 500 to 6000 | 0 |
| Next sample | `obs_gap` | 45 | 5 to 200 | 1 |
| Next sample | `obs_width` | 95 | 20 to 300 | 1 |
| Next sample | `obs_match` | 60 | 10 to 300 | 1 |
| Next sample | `obs_confirm` | 4 | 1 to 50 | 0 |

The lidar parameters exist in competition mode only. `CHASSIS_MAX_WHEEL_RPM` and `CHASSIS_MOVE_MAX_SPEED` stay compile-time as hard limits.

## Rules

- Unknown names, out-of-range values and malformed numbers (`nan`, `1e3`, `2.5x`) are rejected and the value stays as it was.
- A stored value is rounded to the parameter's decimals, so what `/list` shows is exactly what is in use.
- Stopped means the chassis is in velocity mode with no commanded motion.
- All values live in RAM. The H750VB's internal flash is one 128 KB sector that holds the firmware, so it can't store settings. The debug page below remembers changed values and offers to write them back; final values go into the macros.
- Setting `heading_ki` to 0 also clears the integral term, so an old correction doesn't stay in the output.

## Debug page

`tools/agv_console.html` is the interface on top of this console: one offline file, opened in Chrome or Edge, using Web Serial. Its parameter list comes from `/list`, so a new parameter needs no page change. It parses only the firmware's text output. The formats it reads are pinned by `console_test`, `comp_task_test` and `survey_loop_test`.

## Out of scope

- Saving values across power-off.
- Host vision parameters; these stay in `vision/config.py`.
- Arm parameters and jog or teach commands; they need the arm hardware details.
- Locking the console during a run; this belongs with the start button, which doesn't exist yet. The rules ban remote control during a run, so that step must add it.

## Testing

Host tests, written first, against the real chassis and parser wherever possible:

- Changing a parameter changes real chassis or lidar behaviour.
- Rejected input leaves values unchanged.
- Console lines never stop the robot, and `S` on a normal line still does.
- Line endings: CR, LF, UART idle, too-long lines, receive overflow.
- Stopped-only parameters are refused while moving.
- Plot rate, channels and stop.
- Competition commands in a mode 2 build, and mode gating in all three modes.
