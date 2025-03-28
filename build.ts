import { type Build } from 'cmake-ts-gen';

const build: Build = {
    common: {
        project: 'm3d',
        archs: ['x64'],
        variables: [],
        copy: {
            'model3d/m3d.h': 'model3d/m3d.c'
        },
        defines: ['M3D_IMPLEMENTATION'],
        options: [],
        subdirectories: [],
        libraries: {
            'm3d': {
                sources: ['model3d/m3d.c']
            }
        },
        buildDir: 'build',
        buildOutDir: 'libs',
        buildFlags: []
    },
    platforms: {
        win32: {
            windows: {},
            android: {
                archs: ['x86', 'x86_64', 'armeabi-v7a', 'arm64-v8a'],
            }
        },
        linux: {
            linux: {}
        },
        darwin: {
            macos: {}
        }
    }
}

export default build;