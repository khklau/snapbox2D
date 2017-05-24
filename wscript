from waflib import Logs
from waflib.extras.preparation import PreparationContext
from waflib.extras.build_status import BuildStatus

def options(optCtx):
    optCtx.recurse('env')
    optCtx.recurse('Box2D')

def prepare(prepCtx):
    prepCtx.recurse('env')
    prepCtx.recurse('Box2D')

def configure(confCtx):
    confCtx.recurse('env')
    confCtx.recurse('Box2D')

def build(buildCtx):
    status = BuildStatus.init(buildCtx.path.abspath())
    if status.isSuccess() and not(buildCtx.is_install):
	Logs.pprint('NORMAL', 'Build already complete                   :', sep='')
	Logs.pprint('GREEN', 'skipping')
	return
    buildCtx.recurse('Box2D')
    status.setSuccess()
