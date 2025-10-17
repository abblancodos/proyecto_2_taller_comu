project('dspproj2','cpp',
        default_options : ['cpp_std=c++20','warning_level=3'],
        version : '0.0.5')

add_project_arguments('-Wpedantic', '-Wno-deprecated-enum-enum-conversion',language : 'cpp')

qt5 = import('qt5')
# Include PrintSupport in base Qt dependencies
qt5_dep = dependency('qt5', modules: ['Core', 'Gui', 'Widgets', 'PrintSupport'])

# Try different package names first, then fall back to local sources
qcp_dep = dependency(['QCustomPlot', 'qcustomplot'], required: false)
if not qcp_dep.found()
    # Add QCustomPlot headers to moc processing from the start
    moc_headers = [
        'mainwindow.h',
        'qcustomplot.h'
    ]
    # Setup QCustomPlot with local sources
    qcustomplot_inc = include_directories('.')
    qcustomplot_sources = ['qcustomplot.cpp']

    # Compile moc files
    moc_files = qt5.compile_moc(headers: moc_headers,
                                dependencies: qt5_dep)

    # Add the moc files to qcustomplot sources
    qcustomplot_sources += moc_files

    # Create comprehensive QCustomPlot dependency
    qcp_dep = declare_dependency(
        include_directories: qcustomplot_inc,
        sources: qcustomplot_sources,
        dependencies: [
            dependency('Qt5Widgets'),
            dependency('Qt5Core'),
            dependency('Qt5PrintSupport')
        ]
    )
else
    # If system QCustomPlot is found, just compile mainwindow moc
    moc_headers = ['mainwindow.h']
    moc_files = qt5.compile_moc(headers: moc_headers,
                                dependencies: qt5_dep)
endif

uic_sources = ['mainwindow.ui']
uic_files = qt5.compile_ui(sources: uic_sources)

# Find other dependencies
jack_dep = dependency('jack')
sndfile_dep = dependency('sndfile')
boost_dep = dependency('boost', modules : ['program_options'])
fftw_dep = dependency('fftw3f')
all_deps = [
    jack_dep,
    sndfile_dep,
    boost_dep,
    fftw_dep,
    qt5_dep,
    qcp_dep
]

sources = files(
    'main.cpp',
    'jack_client.cpp',
    'dsp_client.cpp',
    'sndfile_thread.cpp',
    'freq_filter.cpp',
    'mainwindow.cpp'
)

# Add only uic_files to sources since moc_files are handled in dependencies
sources += uic_files

executable('dspproj2', sources, dependencies: all_deps)