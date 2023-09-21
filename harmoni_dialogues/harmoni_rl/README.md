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

## References
[Documentation](https://harmoni.readthedocs.io/en/latest/packages/harmoni_rl.html)