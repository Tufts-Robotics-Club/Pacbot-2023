#!/bin/bash

tmux new-session -s PacBot -d

tmux new-window -n "server" -c ~/Git/Pacbot-2023/ 'python3 server_test.py'

tmux new-window -n "comms" -c ~/Git/Pacbot-2023/ 'python3 commsModule.py'

tmux new-window -n "ai" -c ~/Git/Pacbot-2023 'python3 aiModule.py'

tmux attach-session -t PacBot

