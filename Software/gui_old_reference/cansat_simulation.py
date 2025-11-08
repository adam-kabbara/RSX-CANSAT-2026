"""
Enhanced payload simulation with command parsing
"""
import sys
import re

def parse_command(cmd_string):
    """
    Parse commands like: CMD,3114,CX,ON
    Returns: (team_id, command, parameter)
    """
    try:
        parts = cmd_string.split(',')
        if len(parts) >= 4 and parts[0] == "CMD":
            team_id = parts[1]
            command = parts[2]
            parameter = parts[3] if len(parts) > 3 else None
            return team_id, command, parameter
    except:
        pass
    return None, None, None

def handle_command(team_id, command, parameter):
    """
    Handle specific commands and return appropriate response
    """
    responses = {
        "CX": lambda p: f"Telemetry transmission {'started' if p == 'ON' else 'stopped'}",
        "ST": lambda p: f"Time set to {p}",
        "CAL": lambda p: "Altitude calibrated",
        "RR": lambda p: "Processor restart initiated",
        "TEST": lambda p: "Connection test successful",
        "SIM": lambda p: f"Simulation mode {p}",
        "GTLOGS": lambda p: "Log retrieval not implemented in simulation",
        "MEC": lambda p: f"Mechanism command received: {p}",
    }
    
    if command in responses:
        message = responses[command](parameter)
        return f"$MSG:{{F|IDLE}} {message}"
    else:
        return f"$MSG:{{F|IDLE}} Unknown command: {command}"

def main():
    print("$MSG:{F|IDLE} Payload simulation initialized", flush=True)
    
    try:
        while True:
            line = sys.stdin.readline()
            
            if not line:
                break
            
            line = line.strip()
            
            if not line:
                continue
            
            # Parse command
            team_id, command, parameter = parse_command(line)
            
            if command:
                response = handle_command(team_id, command, parameter)
            else:
                response = f"$MSG:{{F|IDLE}} Raw input: {line}"
            
            print(response, flush=True)
    
    except KeyboardInterrupt:
        print("$MSG:{F|IDLE} Simulation stopped by user", flush=True)
    except Exception as e:
        print(f"$E:MSG:{{F|ERROR}} Exception: {e}", flush=True)

if __name__ == "__main__":
    main()