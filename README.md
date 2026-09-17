# 07-Team Repo
Template for team repo

<p align="center">
<img src="./images/thisismyteam.png" width="50%">
</p>
<p align="center">
</p>

## Team links
- [Team Google Drive](https://drive.google.com/drive/u/1/folders/1HSWW8e8HxwHPLZcLo6FfCO6m2fV2Ahi7)
- [Team Board (view only)](https://app.notion.com/p/Team07_ProjectBoard-3da2b21b7a028070acded2745a7e6e01?source=copy_link)

## Organization of this repo

This depends on your set of roles. Try these queries to set up
your initial repo. You can always change this later.

- "What is a good organization of a repo for a team of two electrical engineers, two computer engineers, and one mechanical engineer? The team is undertaking the design and build of an embedded system and mechanicals for and electronic cat feeder project"


As in your individual repo, you can customize your based on your preferred tools (e.g., CAD
software, IDE, etc.)

Example:

```
├── hardware/                  # Electrical Engineering (EE)
│   ├── schematics/            # Circuit diagrams, block diagrams, design files
│   ├── pcb/                   # PCB layout, Gerber files, drill files
│   ├── bom/                   # Bill of Materials (components, suppliers, costs)
│   └── simulation/            # SPICE simulations, power budget models
│
├── firmware/                  # Computer Engineering (CE/Software)
│   ├── src/                   # Source code (.c, .cpp, etc.)
│   ├── include/               # Header files (.h, .hpp)
│   ├── lib/                   # External libraries and vendor drivers
│   ├── tests/                 # Unit tests, integration tests
│   └── build/                 # Build output / configuration files (PlatformIO/CMake)
│
├── mechanical/                # Mechanical Engineering (ME)
│   ├── cad/                   # 3D models (STEP, SLDPRT, Fusion 360 archives)
│   ├── drawings/              # 2D technical drawings, dimensioned PDFs
│   ├── 3d_print/              # STL/3MF files for fast prototyping
│   └── renders/               # Product renders and enclosure mockups
│
├── docs/                      # Shared Project Documentation
│   ├── architecture/          # Pinout tables, system wiring, block diagrams
│   └── datasheets/            # Component PDFs (ICs, sensors, motors)
│
├── scripts/                   # Shared Automation & Utilities
│   ├── flashing/              # Flash/programming scripts
│   └── tests/                 # Hardware-in-the-loop / test automation scripts
│
├── .gitignore                 # Ignore generated files (binaries, Gerber zips, CAD locks)
└── README.md                  # Project overview, setup guides, and team roles
```

