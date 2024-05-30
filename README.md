# noetic-llama

`docker-compose` docker containers required for LCASTOR intent recognition and transcription with whisper.
Currently, `whisperwrapper` needs to be run outside of docker due to problems passing through the microphone, which is why it is symlinked here.

If being run in a VM, set the whisper API url ROS parameter:

`rosparam set /stt/whisper_api_url 192.168.122.1:9000`

If being run on a separate PC to the host, you may need to set these parameters:

`export ROS_MASTER_URI=http://10.68.0.1:11311`

`export ROS_IP=10.68.0.130`

Obviously you should update the IP addresses with the appropirate ones as applicable.

If debugging, you might need to manually make the node transcribe:

`rostopic pub /stt/listening ollamamessages/WhisperListening "listening: True" -1`

There is a bash script provided to start the whisper node, make sure you have compiled, sourced,
and are in the correct directory first.

## TODOs

 - [ ] Make a proper parser for the function calls returned by ollama instead of just using `exec()`, this will allow us to fetch the return value of the functions, and it's also much safer. A grammar has already been made, see [ollamafunctiongrammar.ppeg](/noetic-llama/src/ollamawrapper/src/ollamafunctiongrammar.ppeg), just need to finish the abstract syntax tree parsing (see [parser.py](/noetic-llama/src/ollamawrapper/src/parser.py))
