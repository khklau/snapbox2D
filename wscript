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

__thirdPartySrcDir = 'src'

def options(optCtx):
    optCtx.load('dep_resolver')
    optCtx.recurse('env')
    optCtx.recurse('src')
    optCtx.recurse('test')

def prepare(prepCtx):
    prepCtx.options.dep_base_dir = prepCtx.srcnode.find_dir('..').abspath()
    prepCtx.load('dep_resolver')
    prepCtx.recurse('env')
    prepCtx.recurse('src')
    prepCtx.recurse('test')

def configure(confCtx):
    confCtx.load('dep_resolver')
    status = BuildStatus.init(confCtx.path.abspath())
    if status.isSuccess():
	confCtx.msg('Configuration already complete', 'skipping')
	return
    thirdPartySrcDir = os.path.join(confCtx.path.abspath(), __thirdPartySrcDir)
    os.chdir(thirdPartySrcDir)
    if os.name == 'posix':
	returnCode = subprocess.call([
		'premake4',
		'gmake'])
	if returnCode != 0:
	    confCtx.fatal('Box2D configure failed: %d' % returnCode)
    elif os.name == 'nt':
	# Nothing to do, just use the provided VS solution
	return
    else:
	confCtx.fatal('Unsupported OS %s' % os.name)

def build(buildCtx):
    status = BuildStatus.load(buildCtx.path.abspath())
    if status.isSuccess():
	Logs.pprint('NORMAL', 'Build already complete                   :', sep='')
	Logs.pprint('GREEN', 'skipping')
	return
    if os.name == 'posix':
	thirdPartySrcDir = os.path.join(buildCtx.path.abspath(), __thirdPartySrcDir, 'Build', 'gmake')
	os.chdir(thirdPartySrcDir)
	returnCode = subprocess.call([
		'make',
		'config="release"',
		'install'])
    elif os.name == 'nt':
	thirdPartySrcDir = os.path.join(buildCtx.path.abspath(), __thirdPartySrcDir, 'Build', 'vs2015')
	os.chdir(thirdPartySrcDir)
	returnCode = subprocess.call([
		'devenv.com',
		os.path.join(thirdPartySrcDir, 'Box2D.sln')])
    else:
	confCtx.fatal('Unsupported OS %s' % os.name)
    if returnCode != 0:
	buildCtx.fatal('Box2D build failed: %d' % returnCode)
    status.setSuccess()
