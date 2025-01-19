import { ApplicationConfig } from '@angular/core';
import { provideAnimationsAsync } from '@angular/platform-browser/animations/async';
import { Routes, provideRouter } from '@angular/router';
import { ClusterComponent } from './cluster/cluster.component';
import { LiveCameraComponent } from './cluster/live-camera/live-camera.component';

const routes: Routes = [
  // {path: '', component: ClusterComponent},
  {path: '', component: LiveCameraComponent}
]

export const appConfig: ApplicationConfig = {
  providers: [provideAnimationsAsync(), provideRouter(routes)]
};
