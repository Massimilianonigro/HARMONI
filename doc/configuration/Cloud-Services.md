# Cloud Services

_What is needed to connect to cloud services._




## Setup Amazon Accounts
(If you are planning to use Lex or Polly) Set up AWS account following these steps: 

Create an Amazon Web Services account. AWS has a 1 year free trial that includes a limited number of Polly usages.
Keep this in mind so you do not get charged money at the end of the year.

Once you've created an account, create an IAM user to access Polly and/or Lex.

    * Give the IAM user access permissions to AWS Polly and/or Lex.
    * Give the IAM user access keys. Be sure to save the secret key as you only have one chance to look at it.
    
https://docs.aws.amazon.com/cli/latest/userguide/install-linux.html - Use this link to install the AWS CLI on your PC.

> **_NOTE:_** If you have already your credentials, you can follow the instructions from here!!

Then, in the terminal,
```bash
$ sudo apt-get install awscli
$ aws --version
$ aws configure
# Enter the IAM user access and secret keys here.
```

Note: The docker compose scripts expect your aws keys to be in ~/.aws



To check if your keys configured correctly, you can run the following:
```bash
$ aws configure
```

The result will print as follows (click the Enter button to move forward without editing the keys if they are setup properly):

```bash
AWS Access Key ID [****************XXXX]: 
AWS Secret Access Key [****************XXXX]: 
Default region name [XX-XXXX-X]: 
Default output format [json]: 
```


## Setup Google Accounts
If you are planning on using dialogflow or other google services (like speech to text):

You need to update private-keys.json in ./dockerfiles/config/ with your google credentials.
When you activate google account API, you can create credentials for connecting HARMONI with your account, following this instraction.
Get the API key (https://developers.google.com/maps/documentation/maps-static/get-api-key)
You must have at least one API key associated with your project.

To get an API key:

* Go to the Google Cloud Platform Console.
* Click the project drop-down and select or create the project for which you want to add an API key.
* Click the menu button and select APIs & Services > Credentials.
* On the Credentials page, click + Create Credentials > Create ID client OAuth.
* Click Service Account, fill the input text with your name. Set the role to Editor, and click end.
* Click the menu button and select APIs & Services > Credentials. In the Service Account table (at the bottom of the page) the account you just created will be displayed. Click on edit. Click on add new keys, and save it.
* Click Close.
* The new API key is listed on the Credentials page under API Keys.
(Remember to restrict the API key before using it in production.)
* Save private-keys.json

> **_NOTE:_** If you have already your credentials, you can follow the instructions from here!!


Set credentials on HARMONI:

```bash
$ cd ~/harmoni_catkin_ws/src/HARMONI/dockerfiles/config/
$ nano private-keys.json
# Copy and paste the json content generated in the previous steps.
```


Note: Secret keys and configurations are done locally and mounted to images through the Docker-Compose.yml files. If you run the compose files before generating the keys you may need to delete and recreate the .aws and .gcp folders, as they will have been created with root only permission. Otherwise you should be able to edit them directly.

To check if your keys are mounted correctly after having installed HARMONI, please check if the private-keys.json file exists in ~/.gcp/ by running the following in the container harmoni_full:
```bash
$ cd ~/.gcp
$ ls
```

If the result printed is a file it will be white and it means that everthing is setup correcly, while if it is a folder its colour will be purple (so please follow the steps reported in the Troubleshooting below).
I


## Setup OpenAI ChatGPT [after having installed HARMONI]
Set up your OpenAI account: https://platform.openai.com/

> **_NOTE:_** If you have already your credentials, you can follow the instructions from here!!


Then create in the HARMONI repository (already instealled following instructions in [here](https://harmoni-20.readthedocs.io/en/latest/quickstart/Docker-Quickstart.html#setup) a .env file

Then, in the terminal,
```bash
$ cd HARMONI
$ nano .env
# Enter the IAM user access and secret keys here.
```
and then add:

OPENAI_ORGANIZATION = $ORGANIZATION_ID
OPENAI_API_KEY = $API_KEY

Substitute the $ORGANIZATION_ID and the $API_KEY with your personal credentials that you find the in your OpenAI account setting. 


## Troubleshooting

If something went wrong and you cannot find the private-keys.json file in ~/.gcp/ (but instead you find a folder you can do the following)

```bash
$ cd ~
$ cd .gcp
$ cd private-keys.json
$ nano private-keys.json
# Copy and paste the json content generated in the previous steps.
```

Note that you then need to change the [configuration file](https://github.com/micolspitale93/HARMONI/blob/dev/harmoni2.0/harmoni_detectors/harmoni_stt/config/google_configuration.yaml) of the corresponding google service (speech to text) as follows:
```bash
stt:
  default_param:
    language_id: "en-UK" #"en-US"
    sample_rate: 48000 #16000 #44100 for wav audio file, 16000 for wav file from mic, 48000
    audio_channel: 1
    subscriber_id: "default"
    credential_path: "$(env HOME)/.gcp/private-keys.json/private-keys.json" #path where private keys are mounted, if the credentials mount correctly this should be "$(env HOME)/.gcp/private-keys.json"
    wait_duration: 2
    max_silence: 10 #s

```
