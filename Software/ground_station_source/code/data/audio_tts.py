"""
Cross-platform, non-blocking TTS helper.

Usage:
    tts = AudioTTS()
    tts.speak("Hello world")
"""
import threading
import subprocess
import platform
import shutil


class AudioTTS:
    def __init__(self):
        self._system = platform.system()

        # detect available backends on non-mac/non-windows
        if self._system not in ("Windows", "Darwin"):
            self._has_espeak = shutil.which("espeak") is not None
            self._has_spd = shutil.which("spd-say") is not None
        else:
            self._has_espeak = False
            self._has_spd = False

    def speak(self, text: str):
        threading.Thread(target=self._run, args=(text,), daemon=True).start()

    def _run(self, text: str):
        try:
            if self._system == "Windows":
                safe = text.replace('\\', '\\\\').replace('"', '\\"')
                ps_cmd = f'Add-Type -AssemblyName System.Speech; (New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak("{safe}")'
                creationflags = getattr(subprocess, 'CREATE_NO_WINDOW', 0)
                subprocess.run(["powershell", "-NoProfile", "-Command", ps_cmd], check=False, creationflags=creationflags)
            elif self._system == "Darwin":
                subprocess.run(["say", text], check=False)
            else:
                if self._has_espeak:
                    subprocess.run(["espeak", text], check=False)
                elif self._has_spd:
                    subprocess.run(["spd-say", text], check=False)
        except Exception:
            # best-effort only
            pass
