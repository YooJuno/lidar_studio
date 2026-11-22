# SPDX-License-Identifier: Apache-2.0
"""Pinned original model bundles and their training-time input geometry."""
from dataclasses import dataclass
import hashlib

AUTOWARE_REVISION='494c8171def40bd36cc2feb323e0a5acbfab132b'
AWML_REVISION='9e44a974aae03a196003950898dc909cd0c680ad'
AUTOWARE_URL=f'https://huggingface.co/AutowareFoundation/lidar_centerpoint/resolve/{AUTOWARE_REVISION}/'
TRANSFUSION_REVISION='8f07eb93038bfd62e35d1f78e56a6b3fb76b12aa'
TRANSFUSION_URL=f'https://huggingface.co/AutowareFoundation/lidar_transfusion/resolve/{TRANSFUSION_REVISION}/'
SHORT_URL='https://download.autoware-ml-model-zoo.tier4.jp/autoware-ml/models/centerpoint/centerpoint-shortrange/t4base/v2.0.0/'
DEFAULT_MODELS=('tiny','short_range')


@dataclass(frozen=True)
class Model:
    name: str
    title: str
    folder: str
    minimum: tuple
    maximum: tuple
    voxel: tuple
    grid: int
    max_voxels: int
    features: int
    downsample: int
    default_score: float  # conservative application preset; not vendor calibration
    files: tuple  # local name, URL, SHA-256

    @property
    def fused_name(self):return self.folder+'-portable-v2.onnx'


MODELS={
    'tiny':Model('tiny','CenterPoint tiny / Autoware v4.1','centerpoint-tiny-v4.1',
        (-76.8,-76.8,-4.),(76.8,76.8,6.),(.32,.32,10.),480,40000,9,2,.35,(
            ('pts_voxel_encoder.onnx',AUTOWARE_URL+'tiny/pts_voxel_encoder.onnx','2c53465715c1fd2e9dc5727ef3fca74f4cdf0538f74286b0946e219d0ca5693b'),
            ('pts_backbone_neck_head.onnx',AUTOWARE_URL+'tiny/pts_backbone_neck_head.onnx','9bb0b634f3664bd098ce7d6a3d8a9fb7cc8d9b8252b27f302c71e43316bab551'),
            ('MODEL_CARD.md',AUTOWARE_URL+'README.md','3cbb8f0f196ef999d3406d88b0d349b808e3d826e2a02a3860dcce94762fc4cf'))),
    'short_range':Model('short_range','AWML ShortRange / ConvNeXt 2.0','centerpoint-shortrange-v2.0.0',
        (-51.2,-51.2,-3.),(51.2,51.2,5.),(.16,.16,8.),640,64000,10,4,.50,(
            ('pts_voxel_encoder.onnx',SHORT_URL+'pts_voxel_encoder_centerpoint_short_range.onnx','c5cfa3e282722f0dd82c8d58294fa3001b0b6d61f38bd48c1e52ee76059ffb85'),
            ('pts_backbone_neck_head.onnx',SHORT_URL+'pts_backbone_neck_head_centerpoint_short_range.onnx','6ac920e99a86a3fabc9e92f0d71e81adbdc2906cedd8e6a17ed0d4207159fe93'),
            ('MODEL_CARD.md',f'https://raw.githubusercontent.com/tier4/AWML/{AWML_REVISION}/projects/CenterPoint/docs/CenterPoint-ShortRange/v2/base.md','027d2f29ced8e1bf30c0a017bac020d7b78e6828139d488ffd36ee065ee9f786'),
            ('LICENSE',f'https://raw.githubusercontent.com/tier4/AWML/{AWML_REVISION}/LICENSE','c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4'))),
    'base':Model('base','CenterPoint base / Autoware v4.1','centerpoint-base-v4.1',
        (-76.8,-76.8,-4.),(76.8,76.8,6.),(.32,.32,10.),480,40000,9,1,.50,(
            ('pts_voxel_encoder.onnx',AUTOWARE_URL+'base/pts_voxel_encoder.onnx','dc1a876580d86ee7a341d543f8ade2ede7f43bd032dc5b44155b1f0175405764'),
            ('pts_backbone_neck_head.onnx',AUTOWARE_URL+'base/pts_backbone_neck_head.onnx','3fe7e128955646740c41a25be0c8f141d5a94594fe79d7405fe2a859e391542e'),
            ('MODEL_CARD.md',AUTOWARE_URL+'README.md','3cbb8f0f196ef999d3406d88b0d349b808e3d826e2a02a3860dcce94762fc4cf'),
            ('ml_package.param.yaml',AUTOWARE_URL+'base/ml_package.param.yaml','76f737ae242416fa59035b84cfbae966d1ef2a8c47556d7dea97e6fa8cdae444'))),
    'transfusion':Model('transfusion','Autoware TransFusion LiDAR v2.1','transfusion-lidar-v2.1',
        (-92.16,-92.16,-3.),(92.16,92.16,7.),(.24,.24,10.),768,60000,5,4,.25,(
            ('transfusion.onnx',TRANSFUSION_URL+'transfusion.onnx','1d8f0ee6d59ccc3cca914f9892f6ac8f0a9e35082abb91da183c00e3e2c2718a'),
            ('MODEL_CARD.md',TRANSFUSION_URL+'README.md','5e2a2b88b0a4a2dba378ade580da2d0c601e0be64cd424c458390ff47cb870f1'),
            ('ml_package.param.yaml',TRANSFUSION_URL+'transfusion_ml_package.param.yaml','476f7727adc17a823962f2e09ba23d40f3116c50be48361d98179d054cd131b6'))),
}


def get_model(name):
    try:return MODELS[name]
    except KeyError:raise ValueError('Unknown detection model: '+str(name)) from None


def sha256_file(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):
            digest.update(chunk)
    return digest.hexdigest()
