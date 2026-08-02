// @ts-check

require('reflect-metadata');

// Useful for Electron/NW.js apps as GUI apps on macOS doesn't inherit the `$PATH` define
// in your dotfiles (.bashrc/.bash_profile/.zshrc/etc).
// https://github.com/electron/electron/issues/550#issuecomment-162037357
// https://github.com/eclipse-theia/theia/pull/3534#issuecomment-439689082
require('fix-path')();

// Workaround for https://github.com/electron/electron/issues/9225. Chrome has an issue where
// in certain locales (e.g. PL), image metrics are wrongly computed. We explicitly set the
// LC_NUMERIC to prevent this from happening (selects the numeric formatting category of the
// C locale, http://en.cppreference.com/w/cpp/locale/LC_categories).
if (process.env.LC_ALL) {
    process.env.LC_ALL = 'C';
}
process.env.LC_NUMERIC = 'C';

const { resolve } = require('path');
const theiaAppProjectPath = resolve(__dirname, '..', '..');
process.env.THEIA_APP_PROJECT_PATH = theiaAppProjectPath;
const { default: electronMainApplicationModule } = require('@theia/core/lib/electron-main/electron-main-application-module');
const { ElectronMainApplication, ElectronMainApplicationGlobals } = require('@theia/core/lib/electron-main/electron-main-application');
const { Container } = require('inversify');
const { app } = require('electron');

const config = {
    "applicationName": "Arduino IDE",
    "defaultTheme": {
        "light": "arduino-theme",
        "dark": "arduino-theme-dark"
    },
    "defaultIconTheme": "none",
    "electron": {
        "windowOptions": {},
        "showWindowEarly": true,
        "splashScreenOptions": {},
        "uriScheme": "arduino-ide"
    },
    "defaultLocale": "",
    "validatePreferencesSchema": false,
    "reloadOnReconnect": true,
    "uriScheme": "theia",
    "preferences": {
        "window.title": "${rootName}${activeEditorShort}${appName}",
        "files.autoSave": "afterDelay",
        "editor.minimap.enabled": false,
        "editor.tabSize": 2,
        "editor.scrollBeyondLastLine": false,
        "editor.quickSuggestions": {
            "other": false,
            "comments": false,
            "strings": false
        },
        "editor.maxTokenizationLineLength": 500,
        "editor.bracketPairColorization.enabled": false,
        "breadcrumbs.enabled": false,
        "workbench.tree.renderIndentGuides": "none",
        "explorer.compactFolders": false
    }
};
const isSingleInstance = true;

(async () => {
    if (isSingleInstance && !app.requestSingleInstanceLock()) {
        // There is another instance running, exit now. The other instance will request focus.
        app.quit();
        return;
    }
    
    const container = new Container();
    container.load(electronMainApplicationModule);
    container.bind(ElectronMainApplicationGlobals).toConstantValue({
        THEIA_APP_PROJECT_PATH: theiaAppProjectPath,
        THEIA_BACKEND_MAIN_PATH: resolve(__dirname, 'main.js'),
        THEIA_FRONTEND_HTML_PATH: resolve(__dirname, '..', '..', 'lib', 'frontend', 'index.html'),
        THEIA_SECONDARY_WINDOW_HTML_PATH: resolve(__dirname, '..', '..', 'lib', 'frontend', 'secondary-window.html')
    });
    
    function load(raw) {
        return Promise.resolve(raw.default).then(module =>
            container.load(module)
        );
    }
    
    async function start() {
        const application = container.get(ElectronMainApplication);
        await application.start(config);
    }

    try {
        await load(require('@theia/filesystem/lib/electron-main/electron-main-module'));
        await load(require('arduino-ide-extension/lib/electron-main/arduino-electron-main-module'));
        await start();
    } catch (reason) {
        if (typeof reason !== 'number') {
            console.error('Failed to start the electron application.');
            if (reason) {
                console.error(reason);
            }
        }
        app.quit();
    };
})();
