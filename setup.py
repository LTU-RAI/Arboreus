from setuptools import setup, find_packages

dependencies = [
    'PyYAML',
    'opencv-python',
    'numpy==1.26.4',
    'rasterio==1.4.4',
    'pyproj==3.7.1',
    'mathutils==3.3.0',
    'open3d==0.19.0',
    'lxml==4.8.0',
    'plyfile==1.1.3',
    'jsonschema==3.2.0',
    'requests==2.25.1',
    'tqdm==4.67.3',
    'trimesh==4.12.2',
    'matplotlib==3.5.1',
    'laspy==2.7.0'
]

setup(
    name='arboreus',
    version='0.1.0',
    packages=find_packages(),
    install_requires=dependencies,
    python_requires='>=3.10',  # Specify Python version compatibility
    classifiers=[
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.10',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    author='Fernando Labra Caso',
    author_email='fernando.labra.caso@gmail.com',
    description='Nordic Forest World Generator for Simulation Techniques.',
    long_description=open('README.md').read(),  # Ensure you have a README.md file for long description
    long_description_content_type='text/markdown',
    url='https://github.com/fernand0labra/Arboreus.git',  # Update with your GitHub repo
)