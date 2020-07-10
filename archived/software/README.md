# vent-dev: software

## Getting Started

### (Recommended) Setup pyenv

It is recommended to use [pyenv](https://github.com/pyenv/pyenv) to manage
your local virtual environments.

* Install [pyenv](https://github.com/pyenv/pyenv)
* Install [pyenv-virtualenv](https://github.com/pyenv/pyenv-virtualenv)

Create a new local environment for Python 3.7.7:

```bash
$ pyenv install 3.7.7
$ pyenv virtualenv 3.7.7 ventapp
$ pyenv
```

Note: If you would like to configure your working directory to always use
your `ventapp` virtual environment, do:

```bash
$ cd /path/to/vent-dev/software
$ pyenv local ventapp
```

This will automatically activate your virtual environment when you are in
that directory.

### Setup Poetry

Poetry ([poetry](https://python-poetry.org/docs/)) is used for dependency
management of this project.  You will need to follow the installation steps
on their site to install it.

### Installing Dependencies

To install all dependencies do: `poetry install`

### Executing Application

To run the application do: `poetry run ventapp`

## Development

### Linting

Linting is a great way to ensure code quality and style remain consistent
between developers.  This project uses pyLint with
(Google pystyle)[https://github.com/google/styleguide/blob/gh-pages/pyguide.md]
rules and will enforce code adheres to these rules during a pull request.

To improve development time, you can configure your IDE to run pyLint as
you are developing.  Here are a few links to common IDE's:

* Sublime: https://packagecontrol.io/packages/Pylinter
* Atom: https://atom.io/packages/linter-pylint

To run the pylint commands, do:

```bash
(software) $ pylint ventapp
```

### Testing

Unit testing is done utilizing [pytest](https://docs.pytest.org/en/latest/) and
(tox)[https://tox.readthedocs.io/en/latest/] for automation.

To execute the test suite, run:

```bash
(software) $ tox
```
