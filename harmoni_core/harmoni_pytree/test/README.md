# Using Tests
In order to test any particular leaves or subtrees, execute the below command and check for the status results,
```
rostest <package-name> <test-name>
``` 
For using any of the tests in this directory, `<package-name>` would correspond to `harmoni_pytree` and `<test-name>` can be any of the `*.test` files, which are similar to `*.launch` files. By default the stdout is not printed on the terminal in case of rostests, but can be printed using the flag `--text`.
```
rostest <package-name> <test-name> --text
``` 

## Working of tests
All the tests have a common structure of defining a sub-class of `unittest.TestCase`, in which a `setup` function is defined and some functions starting with `test_*` are defined. On running the test, the setup function is called initially, thereafter the `test_` functions are called. The test result is `SUCCESS`, when there are no assertion failure and the test is completed before timeout(default ~60 secs, which can be changed if default timeout is inappropriate). 

## Expected behaviour of tests
In most of the cases of exception, the tests would give `FAILURE` as result, but there may be some cases wherein the exception is not caught, say, due to KeyBoardInterrupt. In such cases, below listed are the expected ouputs of the test, which can be used for testing the services,
1. bot_test: A response from lex is received based on the message in the request defined in the test file("Hi").
2. face_test: The cordial face(which can be seen on webpage) would show non-default facial expressions.
3. speaker_test: A sentence would be spoken from a pre-recorded audiofile.
4. tts_test: A response from amazon polly service is received based on the message in the request, defined in the test file('message':"I would like to order some flowers").
5. web_test: A webpage with some images and other components is hosted up.
6. face_and_polly_test: A sentence would be spoken along with the corresponding facial expressions on the cordial face on webpage. The sentence is defined as a rosparam in the test file.
7. mic_to_face_test__audiofile: The webpage of cordial face would show expressions of speaking the text fetched from amazon lex service, and speaker would also give output. The input is taken from a pre-recorded audiofile.
8. mic_to_face_test__mic: The webpage of cordial face would show expressions of speaking the text fetched from amazon lex service, and speaker would also give output. The output would be dependent on the intents configured in the lex bot, the the input text detected by deep stt from the microphone.
9. speaker_and_tts_test: Similar outputs as face_and_polly_test, but without the cordial face.
10. mic_and_stt_test: The output would the words uttered by the user in microphone. The result may not be accurate, depending on the model of speech-to-text.

<!-- By htg_sensei -->
