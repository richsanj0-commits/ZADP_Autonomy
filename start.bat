@echo off
rem Starts the ZADP Dump Planner on this computer and opens it in the browser.
cd /d "%~dp0"
start "" http://localhost:8765/zadp_game.html
python -m http.server 8765
