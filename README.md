# AVR Care Companion Prototype

A three-person embedded-systems course project using an ATmega128A at 16 MHz. The initial goal was an interactive companion for older adults living alone, combining proximity interaction, inactivity monitoring, and medication reminders.

## Implemented behavior

- Ultrasonic proximity sensing with servo-driven tail movement.
- Separate FSMs for proximity, inactivity, and medication-related behavior.
- Character LCD facial expressions with priority and temporary overlays.
- Timer-driven system time and incremental LCD updates.
- Medication reminder buzzer and confirmation button.
- UART status and alert messages.

## My contribution

Jaejun Lee designed the overall FSM architecture and implemented ultrasonic sensing, servo tail control, the LCD display code, and system integration. Teammates contributed button/medication-alarm/UART functionality and facial-expression patterns/layout. This was a team project; the complete source is not presented as individual authorship.

## Demonstration scope

Inactivity thresholds were shortened to 15, 30, and 45 seconds for demonstration; some UART messages retain the originally intended hour-based wording. Emergency notification is represented by UART text only. Button input represents medication confirmation; the prototype does not detect actual ingestion. No clinical validation was performed.

## Files and status

- `src/main.c`: archived course-project source, with encoding normalized to UTF-8.
- `docs/initial-idea.txt`: original planning notes in Korean.
- `docs/functional-notes.txt`: original functional notes in Korean.

The planning notes record design intent and may differ from the final source. The archived code has not been rebuilt or retested during this portfolio preparation. This repository does not yet include a complete IDE project, wiring diagram, or build configuration. It is a course prototype archive, not a ready-to-deploy care system.
