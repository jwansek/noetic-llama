# noetic-llama

`docker-compose` docker containers required for LCASTOR intent recognition and transcription with whisper.
Currently, `whisperwrapper` needs to be run outside of docker due to problems passing through the microphone, which is why it is symlinked here.

If being run in a VM, set the whisper API url ROS parameter:

`rosparam set /stt/whisper_api_url 192.168.122.1:9000`

If debugging, you might need to manually make the node transcribe:

`rostopic pub /stt/listening ollamamessages/WhisperListening "listening: True" -1`

Given a python library `capabilities` containing only functions, it inspects this library to generate an initial prompt to tell the 
LLM the possible functions. Then, with a ROS service call, these functions are called:

`rosrun ollamawrapper ollamawrapper`

`rosservice call /ollama_wrapper "input: What the weather like in Seattle right now? What's 14 plus 217? What's 0 + 1?"`

Response:

```
Recieved ollama request 'What the weather like in Seattle right now? What's 14 plus 217? What's 0 + 1?'
get_weather_data(coordinates=get_coordinates_from_city(city_name='Seattle')):
By 'Seattle' I am assuming you mean 'Seattle, WA'
The current temperature is 0.100000°C with an outlook of 'Clear sky'
add(num1=14, num2=217):
14.000000 + 217.000000 = 231.000000
add(num1=0, num2=1):
0.000000 + 1.000000 = 1.000000
```

## TODOs

 - [ ] Make a proper parser for the function calls returned by ollama instead of just using `exec()`, this will allow us to fetch the return value of the functions, and it's also much safer. A grammar has already been made, see [ollamafunctiongrammar.ppeg](/noetic-llama/src/ollamawrapper/src/ollamafunctiongrammar.ppeg), just need to finish the abstract syntax tree parsing (see [parser.py](/noetic-llama/src/ollamawrapper/src/parser.py))
