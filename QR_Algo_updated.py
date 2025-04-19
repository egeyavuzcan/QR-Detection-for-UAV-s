#!/usr/bin/env python3
"""
QR Detection Module for UAVs.

This module provides the QRDetector class to capture video streams, detect QR codes,
and apply perspective transformation.
"""

import cv2
import numpy as np
import argparse


class QRDetector:
    """Class for detecting and decoding QR codes from a video stream."""

    def __init__(self, source: str, output_size: tuple = (800, 800)):
        """
        Initialize the video capture.

        Args:
            source: Video capture source string or device index.
            output_size: Size (width, height) for perspective transformed output.
        """
        self.cap = cv2.VideoCapture(source)
        self.output_size = output_size
        self.decoder = cv2.QRCodeDetector()

    def detect_and_display(self):
        """
        Process the video stream, detect QR codes, apply perspective transform,
        and display the results. Press 'q' to exit.
        """
        while True:
            success, frame = self.cap.read()
            if not success:
                print("Failed to read frame from source.")
                break

            # Detect QR code in frame
            data, points, _ = self.decoder.detectAndDecode(frame)

            if points is not None:
                # Define destination points for perspective transform
                width, height = self.output_size
                points_dst = np.float32([
                    [0, 0], [width, 0], [width, height], [0, height]
                ])
                # Convert detected points to float32
                src_pts = points.reshape(4, 2).astype(np.float32)
                # Compute transform matrix and apply warp
                matrix = cv2.getPerspectiveTransform(src_pts, points_dst)
                transformed = cv2.warpPerspective(frame, matrix, self.output_size)
                print("Applied perspective transformation.")

                # Decode QR from transformed image
                data_transformed, _, _ = self.decoder.detectAndDecode(transformed)
                if data_transformed:
                    print(f"Decoded data after transform: {data_transformed}")
                cv2.imshow("Transformed QR", transformed)

            if data:
                print(f"Decoded data: {data}")
            cv2.imshow("Camera Feed", frame)

            # Exit on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()


def main():
    """Parse command-line arguments and start the QR detection."""
    parser = argparse.ArgumentParser(description="QR Code Detection for UAVs")
    parser.add_argument(
        "--source", type=str, required=True,
        help="Video source (e.g., device index '0' or GStreamer pipeline string)."
    )
    parser.add_argument(
        "--width", type=int, default=800,
        help="Output width of transformed QR."
    )
    parser.add_argument(
        "--height", type=int, default=800,
        help="Output height of transformed QR."
    )
    args = parser.parse_args()
    detector = QRDetector(args.source, (args.width, args.height))
    detector.detect_and_display()


if __name__ == "__main__":
    main()