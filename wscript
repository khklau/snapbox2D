import hashlib
import os
import shutil
import subprocess
import tarfile
import urllib
import zipfile
from waflib import Logs
from waflib.extras.preparation import PreparationContext
from waflib.extras.build_status import BuildStatus
from waflib.extras.filesystem_utils import removeSubdir
from waflib.extras.mirror import MirroredTarFile, MirroredZipFile

def options(optCtx):
    optCtx.load('dep_resolver')
    optCtx.recurse('Box2D')
    optCtx.recurse('env')
    optCtx.recurse('src')
    optCtx.recurse('test')

def prepare(prepCtx):
    prepCtx.options.dep_base_dir = prepCtx.srcnode.find_dir('..').abspath()
    prepCtx.load('dep_resolver')
    prepCtx.recurse('Box2D')
    prepCtx.recurse('env')
    prepCtx.recurse('src')
    prepCtx.recurse('test')

def configure(confCtx):
    confCtx.load('dep_resolver')
    confCtx.recurse('Box2D')
    confCtx.recurse('env')
    confCtx.recurse('src')
    confCtx.recurse('test')

def build(buildCtx):
    buildCtx.recurse('Box2D')
    buildCtx.recurse('env')
    buildCtx.recurse('src')
    buildCtx.recurse('test')
