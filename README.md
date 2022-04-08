# DDP actuator solver

[![Pipeline status](https://gitlab.laas.fr/stack-of-tasks/ddp-actuator-solver/badges/master/pipeline.svg)](https://gitlab.laas.fr/stack-of-tasks/ddp-actuator-solver/commits/master)
[![Coverage report](https://gitlab.laas.fr/stack-of-tasks/ddp-actuator-solver/badges/master/coverage.svg?job=doc-coverage)](https://gepettoweb.laas.fr/doc/stack-of-tasks/ddp-actuator-solver/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/stack-of-tasks/ddp-actuator-solver/master.svg)](https://results.pre-commit.ci/latest/github/stack-of-tasks/ddp-actuator-solver)

iLQRSolver.py is a library used to compute a sequence of control to
reach a specified point with a specified system and cost function.

In order to use the solver, you have to implement a class which describe
the dynamic model of the system and another which describe the cost
function you will use to solve your problem. The attributes and methods
need to have the same names and return the same things.

# dynamic model

This class is the one which describe the system you want to command. It
needs to have particular attributes and methods.

### attributes

Two attributes are mandatory for the solver to work.

    self.stateNumber
    self.commandNumber

stateNumber attribute allow the solver to know how many variable your
state contains.
commandNumber specified the number of commands your system have.
These 2 attributes are supposed to be unsigned integer.
Both these attributes make the Solver as generic as possible.

### methods

Two methods are mandatory for the solver to work.

```python
def computeNextState(self,dt,X,U):
    ''' code here '''
    return nextX

def def computeAllModelDeriv(self,dt,X,U):
    ''' code here '''
    return lx,lxx,lu,luu,lux,lxu
```

$$\begin{aligned}
lx &=& \partial  \end{aligned}$$

The solver class need to be instantiated with the dynamic model of the
system and the cost function you want to apply.

    solver = ILQRSolver(model, costFunction)
