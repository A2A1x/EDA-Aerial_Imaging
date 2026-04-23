# EDA-Aerial_Imaging

A small exploratory data analysis (EDA) repository for aerial and satellite imagery. This repo collects analyses, visualizations, and notebooks intended to help understand image datasets, label distributions, and basic preprocessing steps used in computer vision tasks on aerial images.

## What you'll find here

- Notebooks with EDA and visualization code (Jupyter notebooks)
- Helper scripts for loading and pre-processing imagery
- Example plots and summaries describing dataset composition

## Repository structure

- data/
  - (not included) place dataset files here or add a README in the data folder describing download steps
- notebooks/
  - EDA_notebook.ipynb — main exploratory analysis and visualizations
- src/
  - data_utils.py — dataset loaders and preprocessing helpers
  - viz_utils.py — plotting helpers used by notebooks
- results/
  - figures/ — generated plots

## Getting started

1. Clone the repository:

   git clone https://github.com/A2A1x/EDA-Aerial_Imaging.git

2. Install dependencies (recommended: use a virtual environment):

   python -m venv .venv
   source .venv/bin/activate   # on Windows use `.venv\Scripts\activate`
   pip install -r requirements.txt

If there's no requirements.txt, typical packages used by the notebooks are:

- numpy
- pandas
- matplotlib
- seaborn
- scikit-image
- rasterio (if working with geospatial rasters)
- jupyterlab or notebook

## Running the notebooks

- Start Jupyter Lab / Notebook:

  jupyter lab

- Open notebooks/EDA_notebook.ipynb and run the cells. Update the data path in the notebook to point to your local dataset (see data/ README or the top of the notebook).

## Data

This repository does not include large imagery datasets. Place your local copy of the dataset under the `data/` directory or update paths in `notebooks/` and `src/` to point to your dataset location. If your dataset requires download (API keys or external hosting), add a short script or instructions in `data/README.md`.

## Contributing

Contributions are welcome. Please:

- Open an issue to discuss larger changes
- Create small, focused pull requests
- Add unit tests for new helper functions when possible

## License

This repository is provided under the MIT License. See LICENSE for details (or add one if missing).

## Contact

Author: A2A1x

If you want specific changes to this README (more details about the dataset, sample images, or a quick-start example), tell me what to include and I will update it.
