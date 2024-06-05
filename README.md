# rpi-sx1280 - SX1280 Python library for Raspberry Pi

[![Publish Python 🐍 distribution 📦 to PyPI and TestPyPI](https://github.com/enthusiastics-ltd/rpi-sx1280/actions/workflows/hatch-publish.yml/badge.svg)](https://github.com/enthusiastics-ltd/rpi-sx1280/actions/workflows/hatch-publish.yml)

Easy to install and use, this library provides an essential API for any SX1280 LoRa module connected to your Raspberry Pi via the SPI interface.

## Getting started

In order to use this library install it first:

```shell
pip install rpi-sx1280
```

Now you can use it within your code.

### Basic module initialisation

```python
from rpi_sx1280 import SX1280


spi_idx = 0
spi_cs = 1
busy_gpio_idx = 22
reset_gpio_idx = 17

lora = SX1280(spi_idx, spi_cs, busy_gpio_idx, reset_gpio_idx)
```

## Developers guide

If you'd like to work on the library itself here are a few things you should know.

### Requirements

This library by default uses the following tools, frameworks, etc.:

- `Python 3` - main supported language
- `hatch` - used as project management tool for Python
- `GitHub` - used as version control, issue tracking, docs

#### Setup environment

1. Install python locally with pip. On Linux you can do this via: `sudo apt install python3-dev python3-pip`.
2. Now install `pipx` as it is usually better to install `hatch` in this way to use it globally. `sudo apt install pipx && pipx ensurepath` and re-init the shell session.
3. Ready to start, setup local environment with `hatch env create`.

#### Running tests

#### Building project

#### Updating version 

This library follows the semver principles. You can use `hatch` to update the version with necessary information.

To bump the version use `hatch version <segment>`.

Here you can find the supported segments docs: [hatch.io > supported segments](https://hatch.pypa.io/latest/version/#supported-segments)

#### Publishing

In order to publish the current state of the library you need to create a new tag and push it to the repository.

```shell
git tag -a 0.1.0 -m "Initial release"
git push origin 0.1.0
```

And the GitHub Actions will take care of the rest.
