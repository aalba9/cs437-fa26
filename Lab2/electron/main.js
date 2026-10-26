const { app, BrowserWindow } = require('electron');
const path = require('node:path');

const target = process.env.PICAR_URL || 'http://picarx:5000';

async function createWindow() {
  const window = new BrowserWindow({
    width: 1200,
    height: 900,
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
      sandbox: true,
    },
  });

  window.webContents.setWindowOpenHandler(() => ({ action: 'deny' }));
  window.webContents.on('will-navigate', (event, url) => {
    if (new URL(url).origin !== new URL(target).origin) {
      event.preventDefault();
    }
  });
  const showConnectionPage = () =>
    window.loadFile(path.join(__dirname, 'connection.html'), {
      query: { target },
    });
  window.webContents.on(
    'did-fail-load',
    (_event, _code, _description, url, isMainFrame) => {
      if (isMainFrame && new URL(url).origin === new URL(target).origin) {
        showConnectionPage();
      }
    },
  );
  await showConnectionPage();
  window.loadURL(target);
}

app.whenReady().then(createWindow);
app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});
app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) {
    createWindow();
  }
});
