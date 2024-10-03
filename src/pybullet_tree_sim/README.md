## Installation

#### General use
```
python -m pip install .
```

#### Development
```
python -m pip install -e.
```


### Useful RegEx commands...
For replacing .mtl file paths
```
From: ([a-zA-Z0-9_]*).mtl\n
To:   ../mtl/$1.mtl\n

```