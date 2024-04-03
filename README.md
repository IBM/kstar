# Kstar Planner is an Automated PDDL based planner that includes planners for top-k and top-quality planning computational tasks.

## The codebase consists of multiple planners, for multiple computational problems, roughly divided into two categories:

1. Top-k planning
2. Top-quality planning  
    2.1. Top-quality planning  
    2.2. Unordered top-quality planning  

The planner implements
* Pure K* search based top-k and top-quality planning
* Symmetry based pruning: OK* search
* Partial order reduction: RK* search 


# Quickstart: Using Kstar Planner as a Python package

The KStar Planner can be easily installed as a Python package using Pip for simplified integration.

```bash
pip install kstar-planner
```

Obtaining plans through KStar Planner is made incredibly straightforward and user-friendly. Let us show a few examples. To retrieve 100 top-quality plans with a quality bound value of 1.0, simply provide the PDDL domain and problem paths to the function displayed below.

```python
from kstar_planner import planners
from pathlib import Path

domain_file = Path("your/path/domain.pddl")
problem_file = Path("your/path/problem.pddl")

heuristic = "ipdb(transform=undo_to_origin())"

plans = planners.plan_topq(domain_file=domain_file, problem_file=problem_file, quality_bound=1.0,number_of_plans_bound=100, timeout=30, search_heuristic=heuristic)
print(plans)
```

Obtaining unordered, high-quality plans follows a comparable process to acquiring top-quality plans.

```python
from kstar_planner import planners
from pathlib import Path

domain_file = Path("your/path/domain.pddl")
problem_file = Path("your/path/problem.pddl")

heuristic = "ipdb(transform=undo_to_origin())"

plans = planners.plan_unordered_topq(domain_file=domain_file, problem_file=problem_file, quality_bound=1.0,number_of_plans_bound=100, timeout=30, search_heuristic=heuristic)
print(plans)
```

For those who desire a specific quantity of top-quality plans without specifying a quality bound value, utilize the Top-K Planner.

```python
from kstar_planner import planners
from pathlib import Path

domain_file = Path("your/path/domain.pddl")
problem_file = Path("your/path/problem.pddl")

plans = planners.plan_topk(domain_file=domain_file, problem_file=problem_file, number_of_plans_bound=100, timeout=30)
print(plans)
```

# Building
For building the code please use
```
./build.py 
```

# Running (examples)

## Top-k

K* with lmcut heuristic
```
./fast-downward.py domain.pddl problem.pddl --search "kstar(lmcut(), k=100)"
```

OK* with lmcut heuristic (recommended)
```
./fast-downward.py domain.pddl problem.pddl --symmetries "sym=structural_symmetries(time_bound=0,search_symmetries=oss,stabilize_initial_state=false,keep_operator_symmetries=true)" --search "kstar(lmcut(), k=100, symmetries=sym)"
```

## Top-quality

K* with iPDB heuristic
```
./fast-downward.py domain.pddl problem.pddl --search "kstar(ipdb(), q=1.0)"
```

OK* with iPDB heuristic (recommended)
```
./fast-downward.py domain.pddl problem.pddl --symmetries "sym=structural_symmetries(time_bound=0,search_symmetries=oss,stabilize_initial_state=false,keep_operator_symmetries=true)" --search "kstar(ipdb(), q=1.0, symmetries=sym)"
```

## Unordered Top-quality

K* with iPDB heuristic
```
./fast-downward.py domain.pddl problem.pddl --search "kstar(ipdb(), q=1.0, find_unordered_plans=true)"
```

OK* with iPDB heuristic
```
./fast-downward.py domain.pddl problem.pddl --symmetries "sym=structural_symmetries(time_bound=0,search_symmetries=oss,stabilize_initial_state=false,keep_operator_symmetries=true)" --search "kstar(ipdb(), q=1.0, symmetries=sym, find_unordered_plans=true)"
```

RK* with iPDB heuristic
```
./fast-downward.py domain.pddl problem.pddl --search "kstar(ipdb(), q=1.0, pruning=limited_pruning(pruning=atom_centric_stubborn_sets(use_sibling_shortcut=true, atom_selection_strategy=quick_skip)), find_unordered_plans=true)"
```

ORK* with iPDB heuristic (recommended)
```
./fast-downward.py domain.pddl problem.pddl --symmetries "sym=structural_symmetries(time_bound=0,search_symmetries=oss,stabilize_initial_state=false,keep_operator_symmetries=true)" --search "kstar(ipdb(), q=1.0, symmetries=sym, pruning=limited_pruning(pruning=atom_centric_stubborn_sets(use_sibling_shortcut=true, atom_selection_strategy=quick_skip)), find_unordered_plans=true)"
```

## Additional options
* Optimization of switching K* from A* to EA is controlled by the following parameters:
    * `openlist_inc_percent_lb` (default `1`) 
    * `openlist_inc_percent_ub` (default `5`) 
    * `switch_on_goal` (default `false`)
* Dumping plans:
    * In case only the number of plans is needed, not the actual plans, an option `dump_plans=false` can be used
    * Dumping the plans into separate files can be avoided with `dump_plan_files=false`
    * Dumping the plans into a single JSON file can be done by specifying `json_file_to_dump=<filename>`

# Building the package:
```bash
# Testing locally
pip install tox pytest -e .
tox
# Output wheels
pip install cibuildwheel
CIBW_BEFORE_BUILD="python -m pip install pip Cython --upgrade" \
CIBW_ARCHS_MACOS="universal2" \
CIBW_ARCHS_LINUX="auto64" \
CIBW_ARCHS_WINDOWS="auto64" \
python -m cibuildwheel --platform macos
# Different versions of CPython from https://python.org must be installed; e.g. 3.8, 3.9, 3.10, 3.11
# CI needs a Mac or Windows VMs, or [docker contexts](https://github.com/StefanScherer/windows-docker-machine), to build wheels for those OSes
```

# Using as a package:
```bash
pip install git+https://github.com/IBM/kstar.git
```
Due to the CLI-oriented design, the code must be run using subprocess.
```python
import sys
import subprocess
import logging
from subprocess import SubprocessError

try:
    output = subprocess.check_output([sys.executable, "-m" "kstar_planner.driver.main", "..your args"])
except SubprocessError as err:
    logging.error(err.output.decode())
```

## Citing

### Top-k planning
```
@InProceedings{lee-et-al-socs2023,
  title =        "On K* Search for Top-k Planning",
  author =       "Junkyu Lee and Michael Katz and Shirin Sohrabi",
  booktitle =    "Proceedings of the 16th Annual Symposium on
                  Combinatorial Search (SoCS 2023)",
  publisher =    "{AAAI} Press",
  year =         "2023"
}

@InProceedings{katz-lee-ijcai2023,
  author =       "Michael Katz and Junkyu Lee",
  title =        "K* Search Over Orbit Space for Top-k Planning",
  booktitle =    "Proceedings of the 32nd International Joint
                  Conference on Artificial Intelligence (IJCAI 2023)",
  publisher =    "{IJCAI}",
  year =         "2023"
}

```


### Top-quality planning
```
@InProceedings{katz-lee-socs2023,
  title =        "K* and Partial Order Reduction for Top-quality Planning",
  author =       "Michael Katz and Junkyu Lee",
  booktitle =    "Proceedings of the 16th Annual Symposium on
                  Combinatorial Search (SoCS 2023)",
  publisher =    "{AAAI} Press",
  year =         "2023"
}

```

## Licensing

Kstar Planner is an Automated PDDL based planner that
includes planners for top-k and top-quality planning computational
tasks. Copyright (C) 2023 Junkyu Lee, Michael Katz, IBM Research, USA.
The code extends the Fast Downward planning system. The license for the
extension is specified in the LICENSE file.

# Fast Downward
<img src="misc/images/fast-downward.svg" width="800" alt="Fast Downward">

Fast Downward is a domain-independent classical planning system.

Copyright 2003-2022 Fast Downward contributors (see below).

For further information:
- Fast Downward website: <https://www.fast-downward.org>
- Report a bug or file an issue: <https://issues.fast-downward.org>
- Fast Downward mailing list: <https://groups.google.com/forum/#!forum/fast-downward>
- Fast Downward main repository: <https://github.com/aibasel/downward>


## Tested software versions

This version of Fast Downward has been tested with the following software versions:

| OS           | Python | C++ compiler                                                     | CMake |
| ------------ | ------ | ---------------------------------------------------------------- | ----- |
| Ubuntu 20.04 | 3.8    | GCC 9, GCC 10, Clang 10, Clang 11                                | 3.16  |
| Ubuntu 18.04 | 3.6    | GCC 7, Clang 6                                                   | 3.10  |
| macOS 10.15  | 3.6    | AppleClang 12                                                    | 3.19  |
| Windows 10   | 3.6    | Visual Studio Enterprise 2017 (MSVC 19.16) and 2019 (MSVC 19.28) | 3.19  |

We test LP support with CPLEX 12.9, SoPlex 3.1.1 and Osi 0.107.9.
On Ubuntu, we test both CPLEX and SoPlex. On Windows, we currently
only test CPLEX, and on macOS, we do not test LP solvers (yet).


## Contributors

The following list includes all people that actively contributed to
Fast Downward, i.e. all people that appear in some commits in Fast
Downward's history (see below for a history on how Fast Downward
emerged) or people that influenced the development of such commits.
Currently, this list is sorted by the last year the person has been
active, and in case of ties, by the earliest year the person started
contributing, and finally by last name.

- 2003-2022 Malte Helmert
- 2008-2016, 2018-2022 Gabriele Roeger
- 2010-2022 Jendrik Seipp
- 2010-2011, 2013-2022 Silvan Sievers
- 2012-2022 Florian Pommerening
- 2013, 2015-2022 Salomé Eriksson
- 2018-2022 Patrick Ferber
- 2021-2022 Clemens Büchner
- 2021-2022 Dominik Drexler
- 2022 Remo Christen
- 2015, 2021 Thomas Keller
- 2016-2020 Cedric Geissmann
- 2017-2020 Guillem Francès
- 2018-2020 Augusto B. Corrêa
- 2020 Rik de Graaff
- 2015-2019 Manuel Heusner
- 2017 Daniel Killenberger
- 2016 Yusra Alkhazraji
- 2016 Martin Wehrle
- 2014-2015 Patrick von Reth
- 2009-2014 Erez Karpas
- 2014 Robert P. Goldman
- 2010-2012 Andrew Coles
- 2010, 2012 Patrik Haslum
- 2003-2011 Silvia Richter
- 2009-2011 Emil Keyder
- 2010-2011 Moritz Gronbach
- 2010-2011 Manuela Ortlieb
- 2011 Vidal Alcázar Saiz
- 2011 Michael Katz
- 2011 Raz Nissim
- 2010 Moritz Goebelbecker
- 2007-2009 Matthias Westphal
- 2009 Christian Muise


## History

The current version of Fast Downward is the merger of three different
projects:

- the original version of Fast Downward developed by Malte Helmert
  and Silvia Richter
- LAMA, developed by Silvia Richter and Matthias Westphal based on
  the original Fast Downward
- FD-Tech, a modified version of Fast Downward developed by Erez
  Karpas and Michael Katz based on the original code

In addition to these three main sources, the codebase incorporates
code and features from numerous branches of the Fast Downward codebase
developed for various research papers. The main contributors to these
branches are Malte Helmert, Gabi Röger and Silvia Richter.


## License

The following directory is not part of Fast Downward as covered by
this license:

- ./src/search/ext

For the rest, the following license applies:

```
Fast Downward is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

Fast Downward is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
```
