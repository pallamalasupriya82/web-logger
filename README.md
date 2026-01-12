# Web-Based Data Logger 📊

An embedded data logging solution that hosts its own website for real-time sensor monitoring and data extraction.

## 💡 Key Features
- **Embedded Web Server:** The microcontroller acts as a standalone server (no external cloud needed).
- **Real-Time Dashboard:** Displays live sensor values (Temperature, Humidity, Voltage, etc.) on a web page.
- **CSV Download:** Users can download historical data logs directly from the browser as a `.csv` file for Excel analysis.

## 🛠️ Tech Stack
- **Microcontroller:** ESP32 / ESP8266
- **Web Technologies:** HTML, CSS, JavaScript (AJAX for live updates)
- **Storage:** SPIFFS / LittleFS (Internal Flash Memory)
- **Protocols:** HTTP, REST API

## 📸 Implementation
Instead of using an SD card which requires physical removal, this system stores data in the internal flash memory. A simple "Download" button on the web interface triggers a file stream, allowing the user to save the logs instantly.
