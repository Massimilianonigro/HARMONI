# HARMONI rl

This package wraps custom trained chatrl services that can be used with HARMONI.

## Usage

Dowload the custom model from the HARMONI path:
```  bash
sh ./harmoni_dialogues/harmoni_rl/get_rl_model.sh
```
Choice of model to be used can be specified in harmoni_rl/config/configuration.yaml

## Parameters

Parameters input for the aws lex service: 

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|user_id               |            |        |
|rl_name              |            |        |
|rl_alias             |            |        |
|region_name           |            |        |

## Testing

The module can be tested using
```  bash
rostest harmoni_rl rl.test
```

## Troubleshooting
If an error like that appears give permission to write to the harmoni_models folder (using the command `chmod +777`)
```  bash
This script should be run from the HARMONI directory in order to place the models in a parallel directory
mkdir: cannot create directory ‘harmoni_models’: Permission denied
``` 

## References
[Documentation](https://harmoni.readthedocs.io/en/latest/packages/harmoni_rl.html)