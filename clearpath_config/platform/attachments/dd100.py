# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from clearpath_config.common.types.accessory import Accessory
from clearpath_config.common.types.platform import Platform
from clearpath_config.platform.types.attachment import BaseAttachment, PlatformAttachment
from typing import List


class DD100TopPlate(BaseAttachment):
    PLATFORM = Platform.DD100
    ATTACHMENT_MODEL = '%s.top_plate' % PLATFORM
    PACS = 'pacs'
    MODELS = [PACS]
    PARENT = 'default_mount'
    HEIGHT = 0.1

    def __init__(
            self,
            name: str = ATTACHMENT_MODEL,
            model: str = PACS,
            enabled: bool = BaseAttachment.ENABLED,
            height: float = HEIGHT,
            parent: str = PARENT,
            xyz: List[float] = Accessory.XYZ,
            rpy: List[float] = Accessory.RPY
            ) -> None:
        super().__init__(name, model, enabled, parent, xyz, rpy)
        self.height = height

    def to_dict(self) -> dict:
        d = super().to_dict()
        d['height'] = self.height
        return d

    def from_dict(self, d: dict) -> None:
        super().from_dict(d)
        if 'height' in d:
            self.height = d['height']


# DD100 Attachments
class DD100Attachment(PlatformAttachment):
    PLATFORM = Platform.DD100
    # Top Plates
    TOP_PLATE = DD100TopPlate.ATTACHMENT_MODEL

    TYPES = {
        TOP_PLATE: DD100TopPlate,
    }
