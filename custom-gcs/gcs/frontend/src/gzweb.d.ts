// gzweb (Gazebo's browser viewer, npm "gzweb") ships no type declarations.
// Only the part Sim3DPanel uses -- see node_modules/gzweb/src/SceneManager.ts.
declare module "gzweb" {
  import type { Observable } from "rxjs";

  export interface SceneManagerConfig {
    elementId?: string;
    websocketUrl?: string;
    websocketKey?: string;
    enableLights?: boolean;
  }

  export class SceneManager {
    constructor(config?: SceneManagerConfig);
    destroy(): void;
    resize(): void;
    resetView(): void;
    follow(entityName: string): void;
    thirdPersonFollow(entityName: string): void;
    getConnectionStatusAsObservable(): Observable<boolean>;
  }
}
