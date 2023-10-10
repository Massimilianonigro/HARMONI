# HARMONI Bot

This package wraps different chatbot services that can be used with HARMONI. Currently we support AWS Lex, Google Dialogflow, and ChatGPT. Rasa is a high priority on our roadmap for local chatbot functionality.

You can run this module in the `harmoni_full` container.


## Usage

### AWS Lex
The API for AWS Lex Bot has:
- Request Name: ActionType: DO
- Body: data(str)
- Response:
    - response (int): SUCCESS, or FAILURE 
    - message (str): response from the AWS lex


### Google DialogFlow
The API for Google DialogFlow has:
- Request Name: ActionType: REQUEST
- Body: data(str)
- Response:
    - response (int): SUCCESS, or FAILURE 
    - message (str): response from the dialogflow


### ChatGPT
The API for ChatGPT has:
- Request Name: ActionType: REQUEST
- Body: data(str): input text
- Response:
    - response (int): SUCCESS, or FAILURE 
    - message (str): response from the chatgpt


You can run the service with the following command:
```roslaunch harmoni_bot bot_service.launch```

## Parameters

### AWS Lex 
Parameters input for the aws lex service: 

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|user_id               |   name of the use in your AWS Lex         |str; e.g., "Micol"        |
|bot_name              |   the name of the bot you have created in AWS Lex         |  str; e.g., "QTRobot"      |
|bot_alias             |   the alias name of your published bot in AWS Lex         |  str; e.g., "DemoBot"     |
|region_name           |   the region of your AWS         |  str; e.g., "eu-west-2"      |


### Google DialogFlow 

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|project_id               |  the ID of your project in the Cloud Service from Google        |str; e.g., "agent1-ewhnht"     |
|language              |   the language for your dialogue         |  str; e.g., "it"     |
|session_id             |   the name of your session        |  str; e.g., "test_bot"  |
|credential_path           |   path where your credentials are stored      |  str; e.g., "$(find harmoni_stt)/config/private-keys.json"      |


### OpenAI ChatGPT
The service ChatGPT has no parameters required.

## Testing.



AWS Lex module can be tested using

```  bash
rostest harmoni_bot lex.test
```

ChatGPT module can be tested using

```  bash
rostest harmoni_bot chatgpt.test
```



## References
[Documentation](https://harmoni20.readthedocs.io/en/latest/packages/harmoni_bot.html)