import os, boto3

###

def download(bucket, product: str, target: str = "") -> None:
    files = bucket.objects.filter(Prefix=product)
    if not list(files):
        raise FileNotFoundError(f"Could not find any files for {product}")
    for file in files:
        os.makedirs(os.path.dirname(file.key), exist_ok=True)
        if not os.path.isdir(file.key):
            bucket.download_file(file.key, f"{target}{file.key}")

###

# https://documentation.dataspace.copernicus.eu/APIs/S3.html
# https://eodata-s3keysmanager.dataspace.copernicus.eu/panel/s3-credentials

if __name__ == "__main__":
    session = boto3.session.Session()
    s3 = boto3.resource(
        's3',
        endpoint_url='https://eodata.dataspace.copernicus.eu',
        aws_access_key_id="SPH2Q81L6M78CHJ2RNU4",
        aws_secret_access_key="r8SVhvKMUv8AEhN4Ngt0GvMBmwq8VnPBAjvZ9Yf5",
        region_name='default'
    )

    # s3://eodata/CLMS/landcover_landuse/dominant_leaf_type/clms_vlcc_dominant-leaf-type_europe_10m_yearly_v1/2024/01/01/CLMS_HRLVLCC_DLT_S2024_R10m_E48N47_03035_V01_R00/CLMS_HRLVLCC_DLT_S2024_R10m_E48N47_03035_V01_R00.tif
    # download(s3.Bucket("eodata"), "CLMS/landcover_landuse/dominant_leaf_type/clms_vlcc_dominant-leaf-type_europe_10m_yearly_v1/2024/01/01/CLMS_HRLVLCC_DLT_S2024_R10m_E48N47_03035_V01_R00/CLMS_HRLVLCC_DLT_S2024_R10m_E48N47_03035_V01_R00.tif")

    # s3://eodata/CLMS/landcover_landuse/tree_cover_density/clms_vlcc_tree-cover-density_europe_10m_yearly_v1/2024/01/01/CLMS_HRLVLCC_TCD_S2024_R10m_E48N47_03035_V01_R00/CLMS_HRLVLCC_TCD_S2024_R10m_E48N47_03035_V01_R00.tif
    download(s3.Bucket("eodata"), "CLMS/landcover_landuse/tree_cover_density/clms_vlcc_tree-cover-density_europe_10m_yearly_v1/2024/01/01/CLMS_HRLVLCC_TCD_S2024_R10m_E48N47_03035_V01_R00/CLMS_HRLVLCC_TCD_S2024_R10m_E48N47_03035_V01_R00.tif")
