
## Leaves Available
We have implemented a set of leaves that can be used and easily integrated in your behaviour tree.


### Microphone

This leaf is the harmoni_microphone client which starts (ON) the sensing from the microphone. It has the following blackboard clients:

BlackBoard Client  [writer]:
Client Data
- namespace: SensorNameSpace.microphone.name
- name: SensorNameSpace.microphone.name
- read: set()
- write: {'/result'}

Variables
- /microphone/result: py_trees.common.Status

### Facial Expressions


This leaf is the harmoni_face client which does (DO) the facial expressions. It has the following blackboard clients:

BlackBoard Client 1  [reader]:
Client Data
- namespace: ActuatorNameSpace.face.name
- name: ActuatorNameSpace.face.name
- read:  {'/face_exp'}
- write: set() 

Variables
- /face/face_exp: "[{'start': 1, 'type': 'viseme', 'id': 'POSTALVEOLAR'}]"



BlackBoard Client 2  [reader]:
Client Data
- namespace: PyTreeNameSpace.scene.name
- name: PyTreeNameSpace.scene.name
- read:  {'/nlp'}
- write: set() 

Variables
- /scene/nlp: 0


### TTS AWS Polly

This leaf is the harmoni_tts client which requests (REQUEST) the synthetisation of the text. It has the following blackboard clients:


BlackBoard Client 1 [reader]:
Client Data
- namespace: DialogueNameSpace.bot.name
- name: DialogueNameSpace.bot.name
- read:  {'/result'}
- write: set() 

Variables
- /bot/result: "{'message': 'Hi, my name is QT'}

BlackBoard Client 2 [reader]:
Client Data
- namespace: PyTreeNameSpace.scene.name
- name: PyTreeNameSpace.scene.name
- read:  {'/scene/nlp'}
- write: set() 

Variables
- /scene/nlp: 0


BlackBoard Client 3 [writer]:
Client Data
- namespace: ActuatorNameSpace.tts.name
- name: ActuatorNameSpace.tts.name
- read:  set()
- write: {'/tts/result'}

Variables
- /tts/result: "{
            "audio_frame": audio_frame,
            "audio_data": data_array,
            "behavior_data": str(behaviours),
           }"



### Web
This leaf is the harmoni_web client which does (DO) the visualisation on the web GUI. It has the following blackboard clients:

BlackBoard Client 1 [reader]:
Client Data
- namespace: ActuatorNameSpace.web.name
- name: ActuatorNameSpace.web.name
- read:  {'/web/image'}
- write: set() 

Variables
- /web/image: "[{'component_id':'img_only', 'set_content':'https://www.google.it/images/branding/googlelogo/2x/googlelogo_color_160x56dp.png'},{'component_id':'raccolta_container', 'set_content': ''}]"



If you want to use the API REQUEST instead, you can create ad-hoc web leaf, or just edit the current one. 



### Voice Activity Detection (VAD)

This leaf is the harmoni_vad client which start (ON) the voice activity detection. It has the following blackboard clients:

BlackBoard Client 1 [reader]:
Client Data
- namespace: DetectorNameSpace.vad.name
- name: DetectorNameSpace.vad.name
- read:  set() 
- write: {'/vad/result'}

Variables
- /vad/result: TODO


### Speaker

This leaf is the harmoni_speaker client which play (DO) the audio data or the audio file. It has the following blackboard clients:


BlackBoard Client 1 [reader]:
Client Data
- namespace: ActuatorNameSpace.tts.name
- name: ActuatorNameSpace.tts.name
- read:  {'/tts/result'}
- write: set() 

Variables
- /tts/result:"{
            "audio_frame": audio_frame,
            "audio_data": data_array,
            "behavior_data": str(behaviours),
           }" or "/root/harmoni_catkin_ws/src/HARMONI/harmoni_actuators/harmoni_tts/temp_data/tts.wav"


BlackBoard Client 2 [reader]:
Client Data
- namespace: PyTreeNameSpace.scene.name
- name: PyTreeNameSpace.scene.name
- read:  {'/scene/nlp'}
- write: set() 

Variables
- /scene/nlp: 0


### Sentiment

This leaf is the harmoni_sentiment client which requests (REQUEST) to analyse the sentiment of a text. It has the following blackboard clients:


BlackBoard Client 1 [writer]:
Client Data
- namespace: ActuatorNameSpace.sentiment.name
- name: ActuatorNameSpace.sentiment.name
- read:  set()
- write: {'/sentiment/result'}

Variables
- /sentiment/result:"[{'label': 'LABEL_1', 'score': 0.5187261700630188}]"


BlackBoard Client 2 [reader]:
Client Data
- namespace: DialogueNameSpace.bot.name
- name: DialogueNameSpace.bot.name + '/' + PyTreeNameSpace.trigger.name
- read:  {'/bot/result'}
- write: set() 

Variables
- /bot/result: "{'message': 'i really hate you'}"



### Image AI (YOLO)

This leaf is the harmoni_imageai client which requests (REQUEST) to detect something (in the test example it is to detect a person). It has the following blackboard clients:


BlackBoard Client 1 [writer]:
Client Data
- namespace: DetectorNameSpace.imageai_yolo.name
- name: DetectorNameSpace.imageai_yolo.name
- read:  set()
- write: {'/imageai_yolo/result'}

Variables
- /imageai_yolo/result:"person"


### OpenSmile

This leaf is the harmoni_opensmile client which start (ON) to detect and extract audio features. It has the following blackboard clients:


BlackBoard Client  [writer]:
Client Data
- namespace: DetectorNameSpace.opensmile.name
- name: DetectorNameSpace.opensmile.name
- read: set()
- write: {'/opensmile/result'}

Variables
- /opensmile/result: py_trees.common.Status


### OpenFace

This leaf is the harmoni_openface client which start (ON) to detect and extract facial action units. It has the following blackboard clients:


BlackBoard Client  [writer]:
Client Data
- namespace: DetectorNameSpace.openface.name
- name: DetectorNameSpace.openface.name
- read: set()
- write: {'/openface/result'}

Variables
- /openface/result: py_trees.common.Status


### Reinforcement Learning

This leaf is the harmoni_rl client which request (REQUEST) to make a decision on the following action to take. It has the following blackboard clients:


BlackBoard Client 1 [reader and writer]:
Client Data
- namespace: PyTreeNameSpace.scene.name
- name: PyTreeNameSpace.scene.name
- read:  {'/scene/rl', '/scene/exercise'}
- write: {'/scene/action'}

Variables
- /scene/rl: 1
- /scene/action: 1
- /scene/exercise: 3


BlackBoard Client 2 [writer]:
Client Data
- namespace: DialogueNameSpace.rl.name
- name: DialogueNameSpace.rl.name
- read:  set()
- write: {'/rl/result'}

Variables
- /rl/result: "3" ##action to make


### Lips Sync

This leaf is the harmoni_face client which does (DO) the lips synchronisation. It has the following blackboard clients:


BlackBoard Client 1  [reader]:
Client Data
- namespace: ActuatorNameSpace.tts.name
- name: ActuatorNameSpace.tts.name
- read:  {'/tts/result'}
- write: set() 

Variables
- /tts/result: "{
            "audio_frame": audio_frame,
            "audio_data": data_array,
            "behavior_data": str(behaviours),
           }" or "/root/harmoni_catkin_ws/src/HARMONI/harmoni_actuators/harmoni_tts/temp_data/tts.wav"


### Google STT
This leaf is the harmoni_stt client which requests (REQUEST) to trascribe the audio data. It has the following blackboard clients:


BlackBoard Client 1 [reader and writer]:
Client Data
- namespace: PyTreeNameSpace.scene.name
- name: PyTreeNameSpace.scene.name
- read:  {'/scene/nlp'}
- write: set()

Variables
- /scene/nlp: 0


BlackBoard Client 2 [writer]:
Client Data
- namespace: DetectorNameSpace.stt.name
- name: DetectorNameSpace.stt.name
- read:  set()
- write: {'/stt/result'}

Variables
- /stt/result: "Hi" ##depending on what you are saying via the microphone

### Gesture

This leaf is the harmoni_gesture client which plays (DO) a gesture on the robot. It has the following blackboard clients:


BlackBoard Client 1  [reader]:
Client Data
- namespace: ActuatorNameSpace.gesture.name
- name: ActuatorNameSpace.gesture.name
- read:  {'/gesture/result'}
- write: set() 

Variables
- /gesture/result: "{'gesture':'QT/bye', 'timing': 0.5}" ## in the case of QT robot

### External Speaker

This leaf is the harmoni_speaker client which play (DO) the audio data or the audio file from an external speaker. It has the following blackboard clients:


BlackBoard Client 1 [reader]:
Client Data
- namespace: PyTreeNameSpace.scene.name
- name: PyTreeNameSpace.scene.name
- read:  {'/scene/sound'}
- write: set() 

Variables
- /scene/sound: "/root/harmoni_catkin_ws/src/HARMONI/harmoni_actuators/harmoni_tts/temp_data/tts.wav"

### DeepMind STT

This leaf is the harmoni_stt client which requests (REQUEST) to trascribe the audio data. It has the following blackboard clients:


BlackBoard Client 1 [writer]:
Client Data
- namespace: DetectorNameSpace.stt.name
- name: DetectorNameSpace.stt.name
- read:  set()
- write: {'/stt/result'}

Variables
- /stt/result: "Hi" ##depending on what you are saying via the microphone

### ChatGPT

This leaf is the harmoni_stt client which requests (REQUEST) to chatgpt to complete the dialogues and respond to the user. It has the following blackboard clients:


BlackBoard Client 1 [reader]:
Client Data
- namespace: PyTreeNameSpace.scene.name
- name: PyTreeNameSpace.scene.name
- read:  {'/scene/nlp', '/scene/utterance', '/scene/request'}
- write: set()

Variables
- /scene/nlp: 0
- /scene/utterance: "['*user* Can you help me out with a code?']"
- /scene/request: True


BlackBoard Client 2 [writer]:
Client Data
- namespace: DialogueNameSpace.bot.name
- name: DialogueNameSpace.bot.name
- read:  set()
- write: {'/bot/result', '/bot/feeling', '/bot/sentiment'}

Variables
- /bot/result: "" 
- /bot/feeling: "" 
- /bot/sentiment: "" 

### Camera

This leaf is the harmoni_camera client which starts (ON) the sensing from the videocamera. It has the following blackboard clients:

BlackBoard Client  [writer]:
Client Data
- namespace: SensorNameSpace.camera.name
- name: SensorNameSpace.camera.name
- read: set()
- write: {'/camera/result'}

Variables
- /camera/result: py_trees.common.Status

### Facial Expression Recognition (FER)

This leaf is the harmoni_camera client which starts (ON) the recognition of facial expressions. It has the following blackboard clients:

BlackBoard Client  [writer]:
Client Data
- namespace: DetectorNameSpace.fer.name
- name: DetectorNameSpace.fer.name
- read: set()
- write: {'/fer/result'}

Variables
- /fer/result: py_trees.common.Status

## Testing

Here are the steps that you can follow in order to run a demo of pytree in HARMONI. 

### Example 1: Microphone and STT
This demo show how work a small three with just two behaviours: microphone and stt.

1.  Create a valid google credential of using stt service: [https://cloud.google.com/speech-to-text/docs/before-you-begin#setting_up_your_google_cloud_platform_project](https://cloud.google.com/speech-to-text/docs/before-you-begin#setting_up_your_google_cloud_platform_project)

2.  Run the following command in the container to start the demo:

     ```  bash
     rostest harmoni_pytree mic_and_stt_pytree.test
     ```

You will see some information about the structure of the tree and the way in which pytree engines tick the tree. Some blackboards are used to show the results of the leaves and in particular in backboards stt/result shows the result of the speech to text services.

#### Troubleshooting
If the testing is not working, it is likely that your Google Credentials have not been setup correctly. 

It is likely that you cannot find the private-keys.json file in ~/.gcp/ (but instead you find a folder you can do the following).
To fix this, please follow these instructions:

```bash
$ cd ~
$ cd .gcp
$ cd private-keys.json
$ nano private-keys.json
# Copy and paste the json content generated in the previous steps.
```

### Example 2: TTS and Face
This demo show how work a small three with just two behaviours: tts and face.

1.  Run the following command in the container to start the demo:

     ```  bash
     rostest harmoni_pytree tts_face_lips_pytree.test
     ```
2.  Open the browser at the link: http://172.18.3.4:8081/ , and wait for the face to appear.

You will see the lip-sync of the face (the mouth will move).

#### Troubleshooting

If you get an error, it may be that the AWS credentials have not been setup correctly while building the container.

Please re-set them by running the following instruction within the container:
To check if your keys configured correctly, you can run the following:
```bash
$ aws configure
```

If the result will print as follows (click the Enter button to move forward without editing the keys if they are setup properly), it means that you need to copy and past again the AWS credentials INSIDE the container

```bash
AWS Access Key ID [None]: 
AWS Secret Access Key [None]: 
Default region name [None]: 
Default output format [None]: 
```


### Example 3: One turn conversation
This demo show how work one turn of conversation works.

1.  Run the following command in the container to start the demo:

     ```  bash
     rostest harmoni_pytree mic_stt_bot_tts_speaker.test
     ```
2.  Open the browser at the link: http://172.18.3.4:8081/ , and wait for the face to appear.

You can talk to the robot and hear a response.
