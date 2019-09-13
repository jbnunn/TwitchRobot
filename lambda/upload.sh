#!/bin/bash

zip -g lambda_function.zip lambda_function.py
aws lambda update-function-code --function-name TwitchRobot --zip-file fileb://lambda_function.zip
echo "Uploaded function to AWS Lambda"
