# [Twitch Robot](./README.md) > AWS Lambda Function and Alexa Skills Kit (ASK) Skill

This should take less than 30 minutes.

## Steps: Lambda

### Setup the AWS CLI

You'll use the AWS Command Line Interface (AWS CLI) to create and update your Lambda function. Follow [https://docs.aws.amazon.com/cli/latest/userguide/cli-chap-install.html](https://docs.aws.amazon.com/cli/latest/userguide/cli-chap-install.html) to setup your CLI. Don't forget to [configure](https://docs.aws.amazon.com/cli/latest/userguide/cli-chap-configure.html#cli-quick-configuration) the CLI with your AWS credentials.

### Create the function

You'll need to create a [Lambda deployment package](https://docs.aws.amazon.com/lambda/latest/dg/lambda-python-how-to-create-deployment-package.html#python-package-dependencies), which will bundle up your function code (`lambda_function.py`) and required dependencies.

1. Visit the [Lambda dashboard to create your function](https://console.aws.amazon.com/lambda/home?region=us-east-1#/create/function). (Note, this takes you to the `us-east-1` region, you can build in any available region you choose).

2. Choose "Author from Scratch", give your Lambda function a name, and choose the "Python 3.7" runtime.

3. Under "Permissions", choose "Create a new role with basic Lambda permissions." This will give your function permissions to execute and access CloudWatch Logs. 

4. Click the **Create Function** button.

5. When your function has been created, make note of the function name and Lambda Amazon Resource Name (ARN) near the top of the page. You'll need these later.

#### Install the dependencies

    $ cd lambda
    $ pip install --target ./package AWSIoTPythonSDK 
    $ pip install --target ./package ask-sdk-core

#### Zip the dependencies

    $ cd package
    $ zip -r9 ${OLDPWD}/lambda_function.zip .

#### Add your Lambda code to the zip file

    $ cd $OLDPWD
    $ zip -g lambda_function.zip lambda_function.py

#### Add your certs to the zip file

You should have copied your AWS IoT certificates from the steps in [Part 3](./Part3-IoT.md) to your `/lambda/certs/` folder. Add these to your zip file:

    $ zip -g lambda_function.zip certs/*

#### Upload the code to your Lambda function

__Note: Replace `TwitchRobot` below with the name of your AWS Lambda function.__

    $ aws lambda update-function-code --function-name TwitchRobot --zip-file fileb://lambda_function.zip

Or, in one step:

    $ ./upload.sh

(You may need to run `chmod +x upload.sh` before running this for the first time)

#### Future code edits

When you make updates to your Lambda function code, you'll just need to run

    $ zip -g lambda_function.zip lambda_function.py

to add your latest changes to your zip file. Then reupload with

    $ aws lambda update-function-code --function-name <your-function-name> --zip-file fileb://lambda_function.zip

### Setup a Lambda environment variable for your AWS IoT Endpoint

Visit the [Lambda dashboard](https://console.aws.amazon.com/lambda/home?region=us-east-1#/functions) and go to your function. Scroll to the Environment Variables and add a key of "AWS_IOT_ENDPOINT" and a value of the endpoint you created in Part 4.

### Add an Alexa Skills Kit Trigger

Scroll back to the top of your function page, and click the **Add Trigger** button. Choose "Alexa Skills Kit". For "Skill ID verification," choose "Disable" for now. You can come back after you add your skill and enable this. Click the **Add** button to save your trigger.

Copy your ARN again from the top right of the page. Then click the **Save** button to save your Lambda function.

## Steps: ASK

### Login or Create an Amazon Developer Account

If you haven't already [create an Amazon Developer Account](https://developer.amazon.com/home.html). Then login and [visit your Alexa Developer Console](https://developer.amazon.com/alexa/console/ask).

### Create a ASK skill

[Create a new skill](https://developer.amazon.com/alexa/console/ask/create-new-skill). Give your skill a name, and choose "Custom" for the model and "Provision Your Own" for the type. 

### Update the Interaction Model

Click "JSON Editor" under __Interaction Model__, and paste the contents of [en-US.json](./ask/model/en-US.json) into the JSON Editor. Click the **Save Model** button.

### Update the Endpoint

Click "Endpoint" under __Interaction Model__ and paste the Lambda ARN you previously copied into the "Default Region"

### Update the Invocation Name

Click "Invocation" under __Interaction Model__ and give your skill an invocation name, like "twitch robot." This will be the phrase you use to initiate the skill on your Echo devices.

### Build the Model

Now click the **Build Model** button to save and build your model.

### On to the next step...

You're now ready for the final section, [Controlling the Robot](./Part6-Control.md).