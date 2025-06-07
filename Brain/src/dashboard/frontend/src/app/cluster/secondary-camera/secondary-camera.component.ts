import { CommonModule } from '@angular/common';
import { Component } from '@angular/core';
import { Subscription } from 'rxjs';
import { WebSocketService } from '../../webSocket/web-socket.service';

@Component({
  selector: 'app-secondary-camera',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './secondary-camera.component.html',
  styleUrl: './secondary-camera.component.css'
})
export class SecondaryCameraComponent {
  public image: string | undefined;
  public loading: boolean = true;
  private canvasSize: number[] = [304, 171];
  private cameraSubscription: Subscription | undefined;

  constructor(private webSocketService: WebSocketService) { }

  ngOnInit() {
    this.image = this.createBlackImage();

    this.cameraSubscription = this.webSocketService.receiveSecondaryCamera().subscribe(
      (message) => {
        this.image = `data:image/png;base64,${message.value}`;
        this.loading = false;
      },
      (error) => {
        this.image = this.createBlackImage();
        this.loading = true;
        console.error('Error receiving secondary camera feed:', error);
      }
    );
  }

  ngOnDestroy() {
    if (this.cameraSubscription) {
      this.cameraSubscription.unsubscribe();
    }
  }

  createBlackImage(): string {
    const canvas = document.createElement('canvas');
    canvas.width = this.canvasSize[0];
    canvas.height = this.canvasSize[1];
    const ctx = canvas.getContext('2d');
    if (ctx) {
      ctx.fillStyle = 'black';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
    }
    return canvas.toDataURL('image/png');
  }
} 